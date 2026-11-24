#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <SDL2/SDL.h>
#include "sim/SimState.hpp"
#include "sim/Physics.hpp"
#include "sim/Game.hpp"
#include "sim/Renderer.hpp"
#include "sim/InputHandler.hpp"
#include "sim/Scheduler.hpp"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <vector>
#include <windows.h>

// If the sim crashes, write where (module base and return addresses) to stderr and crash.txt next to
// the program, so `addr2line -e vex_sim.exe <address - base + 0x140000000>` can name the line
static LONG WINAPI crashReport(EXCEPTION_POINTERS* e) {
    const DWORD code = e->ExceptionRecord->ExceptionCode;  // real crashes only (access violation, stack overflow and the like);
    if (code < 0xC0000000u || code >= 0xD0000000u) return EXCEPTION_CONTINUE_SEARCH;  // C++ exceptions are caught and handled  // C++ exceptions and notices (the sim stops the robot program with one): not crashes
    void* fr[40];
    USHORT n = CaptureStackBackTrace(0, 40, fr, nullptr);
    FILE* f = std::fopen("crash.txt", "w");
    for (FILE* out : {stderr, f}) {
        if (!out) continue;
        std::fprintf(out, "CRASH code %lx at %p, module base %p\n", e->ExceptionRecord->ExceptionCode,
                     e->ExceptionRecord->ExceptionAddress, (void*)GetModuleHandleA(nullptr));
        for (int i = 0; i < n; ++i) std::fprintf(out, "  frame %p\n", fr[i]);
        std::fflush(out);
    }
    if (f) std::fclose(f);
    return EXCEPTION_CONTINUE_SEARCH;
}

// ─── CLI arg parsing ──────────────────────────────────────────────────────────

static std::vector<int> parsePorts(const char* s) {
    std::vector<int> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        int v = std::stoi(tok);
        if (v != 0) out.push_back(v);
    }
    return out;
}

static const char* argval(int argc, char** argv, const char* flag, const char* def) {
    for (int i = 1; i + 1 < argc; ++i)
        if (std::strcmp(argv[i], flag) == 0) return argv[i + 1];
    return def;
}

static const double PX_PER_IN = 720.0 / 140.41;

static sim::RobotConfig buildConfig(int argc, char** argv) {
    // Helpers — convert to the values we need
    auto dbl  = [&](const char* flag, double def) {
        return std::stod(argval(argc, argv, flag, std::to_string(def).c_str()));
    };
    auto ports = [&](const char* flag, const char* def) -> std::array<int,4> {
        auto v = parsePorts(argval(argc, argv, flag, def));
        std::array<int,4> a = {0,0,0,0};
        for (int i = 0; i < (int)v.size() && i < 4; ++i) a[i] = v[i];
        return a;
    };

    sim::RobotConfig cfg;

    // Physical — convert inches to pixels (1 in = 5 px)
    double wheel_radius_in        = dbl("--wheel-radius",    1.625);
    double track_width_in         = dbl("--track-width",     12.0);
    double tracking_wheel_radius  = dbl("--tracking-radius", 1.0);
    double robot_width_in         = dbl("--robot-width",     8.0);
    double robot_height_in        = dbl("--robot-height",    8.0);

    cfg.wheel_radius_px           = wheel_radius_in        * PX_PER_IN;
    cfg.track_width_px            = track_width_in         * PX_PER_IN;
    cfg.tracking_wheel_radius_px  = tracking_wheel_radius  * PX_PER_IN;
    cfg.robot_half_w              = (robot_width_in  / 2.0) * PX_PER_IN;
    cfg.robot_half_h              = (robot_height_in / 2.0) * PX_PER_IN;

    cfg.max_rpm             = dbl("--max-rpm",      600.0);
    cfg.gear_ratio          = dbl("--gear-ratio",   1.0);
    cfg.gear_friction_coeff = dbl("--gear-friction", 0.0);
    cfg.field_w    = 720.0;
    cfg.field_h    = 720.0;

    // Inertia — tau = m * v_max² / (4 * N * P)
    // v_max uses the effective wheel RPM (motor RPM × gear ratio)
    double mass_kg         = dbl("--mass",         10.0);
    double drive_motors    = dbl("--drive-motors",  6.0);
    double r_m             = (cfg.wheel_radius_px / PX_PER_IN) * 0.0254;
    double v_max           = (cfg.max_rpm * cfg.gear_ratio / 60.0) * 2.0 * M_PI * r_m;
    cfg.accel_time_constant = (mass_kg * v_max * v_max) / (4.0 * drive_motors * 11.0);

    // Motor ports
    cfg.left_ports  = ports("--left-ports",  "4,5,6");
    cfg.right_ports = ports("--right-ports", "1,2,3");

    // Tracking wheel ports (0 = disabled)
    auto iarg = [&](const char* flag, int def) {
        return std::stoi(argval(argc, argv, flag, std::to_string(def).c_str()));
    };
    cfg.tracking_left_port  = iarg("--tracking-left",  0);
    cfg.tracking_right_port = iarg("--tracking-right", 0);
    cfg.tracking_mid_port   = iarg("--tracking-mid",   0);

    return cfg;
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    AddVectoredExceptionHandler(0, crashReport);
    std::srand((unsigned)std::time(nullptr));  // team draws and RANDOMIZE differ every run
    SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, "permonitorv2");  // without this Windows stretches the window on a scaled display and it looks blurry
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    constexpr int FIELD_W = 720;
    constexpr int FIELD_H = 720;
    constexpr int WIN_W   = FIELD_W * 2 + sim::SIDEBAR_W;   // 2D field, 3D view, then the controls sidebar
    constexpr int WIN_H   = FIELD_H + 80;

    // Open the window as large as possible while it still fits on the screen (the picture
    // is scaled to the window, so the sidebar stays visible on smaller displays)
    // Open on the second screen when there is one
    const int display = SDL_GetNumVideoDisplays() > 1 ? 1 : 0;
    int startW = WIN_W, startH = WIN_H;
    SDL_Rect usable;
    if (SDL_GetDisplayUsableBounds(display, &usable) == 0) {
        double fit = std::min({1.0, (usable.w - 40) / (double)WIN_W, (usable.h - 60) / (double)WIN_H});
        startW = (int)(WIN_W * fit);
        startH = (int)(WIN_H * fit);
    }

    SDL_Window* window = SDL_CreateWindow(
        "VEX V5 Simulator  |  F6=Drive  F7=Disable  R=Reset  C=Camera  F11=Full screen  ESC=Quit",
        SDL_WINDOWPOS_CENTERED_DISPLAY(display), SDL_WINDOWPOS_CENTERED_DISPLAY(display),
        startW, startH,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI
    );
    SDL_SetWindowMinimumSize(window, WIN_W / 2, WIN_H / 2);
    // The console (the terminal the sim prints to) goes on the same screen, small, along the bottom and always on top
    if (HWND con = GetConsoleWindow()) {
        SDL_Rect db;
        if (SDL_GetDisplayBounds(display, &db) == 0) SetWindowPos(con, HWND_TOPMOST, db.x, db.y + db.h - 260, 760, 220, SWP_NOACTIVATE | SWP_SHOWWINDOW);
    }
    if (!window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* sdl_ren = SDL_CreateRenderer(
        window, -1,
        SDL_RENDERER_ACCELERATED  // no vsync: waiting for the 60 Hz refresh held most frames to 33 ms and added input lag
    );
    if (!sdl_ren) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    // Draw at the original 720-wide layout and let SDL scale it to any window size,
    // keeping the proportions (black bars fill the rest). Mouse positions are scaled back too.
    SDL_RenderSetLogicalSize(sdl_ren, WIN_W, WIN_H);

    sim::RobotConfig cfg = buildConfig(argc, argv);

    // Pre-set reversed flags from signed port numbers so Physics and stubs
    // agree before initialize() runs.  Negative port = reversed motor.
    for (int port : cfg.left_ports) {
        if (port == 0) break;
        if (port < 0) sim::SimState::get().motors[-port].reversed.store(true);
    }
    for (int port : cfg.right_ports) {
        if (port == 0) break;
        if (port < 0) sim::SimState::get().motors[-port].reversed.store(true);
    }

    // Start pose — coordinates in inches from field center (LemLib convention:
    // X right, Y up, heading 0° = North/up, clockwise-positive).
    // Converted to SDL pixels: SDL_y is flipped, sim heading 0 = East (+X).
    {
        auto dbl = [&](const char* flag, double def) {
            return std::stod(argval(argc, argv, flag, std::to_string(def).c_str()));
        };
        double sx  = dbl("--start-x",       0.0);   // inches from center
        double sy  = dbl("--start-y",       0.0);
        double shd = dbl("--start-heading", 0.0);   // degrees, 0=North CW+

        const double S = FIELD_W / 140.41;
        auto& state = sim::SimState::get();
        std::lock_guard<std::mutex> lock(state.pose_mutex);
        state.pose.x       = FIELD_W / 2.0 + sx * S;
        state.pose.y       = FIELD_H / 2.0 - sy * S;          // flip Y
        state.pose.heading = (shd - 90.0) * M_PI / 180.0;     // to sim radians
        state.pose.heading_accumulated = 0.0;
    }

    sim::Physics     physics(cfg);
    sim::Renderer    renderer(sdl_ren, FIELD_W, FIELD_H, cfg);
    sim::InputHandler input;

    printf("VEX Sim ready in driver control. F6 = drive, F7 = disable.\n");
    printf("Drive: WASD or arrow keys (arcade)\n");
    printf("E/Q lift up/down, Space close claw, F open claw, Shift slow mode, C/V camera\n");

    // Start in driver control (opcontrol runs right after initialize)
    sim::SimState::get().comp_mode.store(sim::CompMode::DISABLED);  // the start screen shows first; PLAY starts the match
    sim::Scheduler::get().start();

    // Testing aids: VEXSIM_PILOT=1 lets the CPU drive the player's robot, VEXSIM_AUTOPLAY=1 presses PLAY
    if (std::getenv("VEXSIM_PILOT")) { sim::game().set.pilot = std::atoi(std::getenv("VEXSIM_PILOT")); sim::game().botsDirty = true; }
    if (const char* sv = std::getenv("VEXSIM_START")) { sim::game().set.startSecs = std::clamp(std::atoi(sv), 10, 600); sim::g_matchLen = sim::game().set.startSecs; sim::game().matchT = sim::g_matchLen; }  // testing: START TIME in seconds
    if (std::getenv("VEXSIM_LIFT")) { const std::string ls = std::getenv("VEXSIM_LIFT"); for (int k = 0; k < 4; ++k) sim::game().set.liftBot[k] = ls[ls.size() == 4 ? k : 0] - '0'; }
    if (const char* pg = std::getenv("VEXSIM_PAGE")) {  // testing: open a start-screen page
        auto& bg = sim::game(); sim::bracketNew(bg);
        if (std::getenv("VEXSIM_MIX")) for (auto& t : bg.teams) { t.rpm = 1 + std::rand() % 2; t.lift = std::rand() % 3; t.omni = false; t.yel = std::rand() & 1; }
        const int pn = std::atoi(pg);
        if (pn == 6) { sim::qualBuild(bg); if (std::getenv("VEXSIM_QVIEW")) { bg.qView = 1; for (int k = 0; k < 20; ++k) { const int r = 60 + std::rand() % 60, bl = k % 7 == 3 ? r : 60 + std::rand() % 60; bg.qRes[k] = {{r, bl, r > bl ? 0 : r < bl ? 1 : 2}}; } bg.qDone = 20; } }
        else if (pn == 1 || pn == 3 || pn == 4 || pn == 5) { sim::qualBuild(bg); sim::formAlliances(bg); }
        bg.dashPage = pn;
        if (pn == 5) sim::loadAllTime(bg);
    }
    if (std::getenv("VEXSIM_BRACKET")) {  // testing: simulate the whole event (qualification, then elimination); VEXSIM_MIX gives every team a random rpm, lift and wheels; VEXSIM_QMATCHES=<n> plays only n qualification matches
        setvbuf(stderr, nullptr, _IONBF, 0);
        auto& bg = sim::game(); sim::bracketNew(bg);
        if (std::getenv("VEXSIM_MIX")) for (auto& t : bg.teams) { t.rpm = 1 + std::rand() % 2; t.lift = std::rand() % 3; t.omni = false; }
        if (const char* tl = std::getenv("VEXSIM_TEAMLIFT")) for (auto& t : bg.teams) t.lift = std::atoi(tl);
        sim::qualBuild(bg);
        if (const char* qn = std::getenv("VEXSIM_QMATCHES")) bg.qSched.resize(std::min<size_t>(bg.qSched.size(), (size_t)std::max(1, std::atoi(qn))));
        bg.simAll = true; bg.autoAll = true; sim::qualStart(bg, true);
    }
    if (const char* qn = std::getenv("VEXSIM_QSIMN")) {  // testing: SIM NEXT <n> on a fresh qualification
        setvbuf(stderr, nullptr, _IONBF, 0);
        auto& bg = sim::game(); sim::bracketNew(bg); sim::qualBuild(bg); bg.qCount = std::atoi(qn);
        sim::dashClick(bg, sim::D_QSIMN, 1);
    }
    if (std::getenv("VEXSIM_OMNI")) sim::game().set.omniBot = {{true, true, true, true}};  // testing: all-omni drawing
    if (std::getenv("VEXSIM_RPM")) { const int rv = std::atoi(std::getenv("VEXSIM_RPM")); sim::game().set.rpmBot = {{rv, rv, rv, rv}}; }
    if (const char* hv = std::getenv("VEXSIM_HUD")) sim::game().set.hud = std::atoi(hv);  // testing: scoreboard and timer bits
    if (std::getenv("VEXSIM_AUTOPLAY")) sim::game().playRequested = true;
    uint32_t last_ticks = SDL_GetTicks();
    // Testing aids: VEXSIM_FIXED_DT=<seconds> steps the physics by that much every frame whatever the
    // frame rate; VEXSIM_TRACE=1 prints the robot, lift, Toggles and red score every 0.1 s to stderr.
    const char* fixedEnv = std::getenv("VEXSIM_FIXED_DT");
    const double fixedDt = fixedEnv ? std::atof(fixedEnv) : 0.0;
    const bool trace = std::getenv("VEXSIM_TRACE") != nullptr;
    uint32_t nextTraceMs = 0;

    while (sim::SimState::get().running.load()) {
        uint32_t now = SDL_GetTicks();
        double dt    = (now - last_ticks) / 1000.0;
        last_ticks   = now;
        dt = std::min(dt, 0.05);
        if (fixedDt > 0.0) {
            dt = fixedDt;
            SDL_Delay(4);  // give the robot program's threads time to run between steps
        }

        if (!input.poll(sim::SimState::get())) break;

        if (sim::game().fastSim) {
            // A simulated tournament match: as many 20 ms steps as fit in about 30 ms, then one frame
            const uint32_t t0 = SDL_GetTicks();
            while (sim::game().fastSim && SDL_GetTicks() - t0 < 30) physics.step(0.02);
        } else {
            physics.step(dt);
        }
        if (trace) {
            auto& st = sim::SimState::get();
            uint32_t ms = st.sim_millis.load();
            if (ms >= nextTraceMs) {
                nextTraceMs = ms + 100;
                const sim::Game& g = sim::game();
                fprintf(stderr, "TR t %.2f mode %d clock %.0f red %d blue %d lift %.1f pose (%.0f,%.0f) |", ms / 1000.0, (int)st.comp_mode.load(), g.matchT, g.red, g.blue, g.liftIn,
                        st.pose.x / sim::PX_IN, (720.0 - st.pose.y) / sim::PX_IN);
                for (int k = 0; k < sim::BOT_N; ++k) {
                    const sim::Bot& b = g.bots[k];
                    fprintf(stderr, " B%d%s (%.0f,%.0f h%.0f cyc%d tk%d.%d c%d g%d p%d l%.0f)", k, b.on ? "" : "-", b.x / sim::PX_IN, (720.0 - b.y) / sim::PX_IN,
                            -b.h * 180.0 / M_PI, b.cyc, b.task, b.phase, b.carry, b.goal, b.placed, sim::dr4bLiftIn(b.liftDeg));
                }
                fprintf(stderr, " | sticks %d %d btn %x motors %d %d", st.controller.leftY.load(), st.controller.rightX.load(), (unsigned)st.controller.buttons.load(), st.motors[11].voltage.load() + st.motors[11].millivolts.load(), st.motors[20].voltage.load() + st.motors[20].millivolts.load());
                fprintf(stderr, " | tipped %d pitch %.2f roll %.2f held %d lift %.1f", (int)g.tipped, g.pitch, g.roll, g.held, g.liftIn);
                { const sim::Bot& pb = g.pilot; fprintf(stderr, " | PS w%.1f bk%.1f wd%.1f tg%d sk%d rp%.1f esc%.1f", pb.wait, pb.backT, pb.wanderT, pb.target, pb.skip, pb.replanT, pb.escapeT); }
                { const sim::Bot& pb = g.pilot; fprintf(stderr, " | P%s (%.0f,%.0f h%.0f cyc%d tk%d.%d c%d g%d l%.0f)", pb.on ? "" : "-", pb.x / sim::PX_IN, (720.0 - pb.y) / sim::PX_IN, -pb.h * 180.0 / M_PI, pb.cyc, pb.task, pb.phase, pb.carry, pb.goal, sim::dr4bLiftIn(pb.liftDeg)); }
                fprintf(stderr, " | minors %d %d dq %d%d stk", g.minors[0], g.minors[1], (int)g.dq[0], (int)g.dq[1]);
                for (int k = 0; k < 9; ++k) fprintf(stderr, " %zu", g.stack[k].size());
                fprintf(stderr, "\n");
            }
        }
        if (std::getenv("VEXSIM_LIFTHOLD")) {  // testing: hold L1 (lift up) for the first VEXSIM_LIFTHOLD seconds of the match
            sim::Game& gl = sim::game();
            const double held = sim::g_matchLen - gl.matchT;
            sim::SimState::get().controller.buttons.store(held >= 0.0 && held < std::atof(std::getenv("VEXSIM_LIFTHOLD")) ? (1u << sim::PILOT_BTN_L1) : 0u);
        }
        if (const char* ea = std::getenv("VEXSIM_END_AT")) { if (sim::g_matchLen - sim::game().matchT >= std::atof(ea) && !sim::game().dash && !sim::game().matchOver) sim::game().endRequested = true; }  // testing: press F8 after this many match seconds
        if (const char* sk = std::getenv("VEXSIM_SKIP_AT")) { static bool done = false; if (!done && sim::g_matchLen - sim::game().matchT >= std::atof(sk) && !sim::game().dash) { done = true; sim::game().skipTo = 15.0; } }  // testing: press F9 after this many match seconds
        if (const char* cm = std::getenv("VEXSIM_CAM")) sim::game().camMode = std::atoi(cm);
        renderer.draw(sim::SimState::get());
    }

    sim::SimState::get().running.store(false);
    sim::Scheduler::get().shutdown();

    SDL_DestroyRenderer(sdl_ren);
    SDL_DestroyWindow(window);
    SDL_Quit();
    std::quick_exit(0);
}
