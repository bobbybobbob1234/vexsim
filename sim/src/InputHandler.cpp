#include "sim/InputHandler.hpp"
#include "pros/misc.h"
#include "sim/Game.hpp"
#include <algorithm>

namespace sim {

InputHandler::InputHandler() = default;

bool InputHandler::poll(SimState& state) {
    SDL_Event event;

    // Snapshot previous buttons for new-press detection
    state.controller.prev_buttons.store(state.controller.buttons.load());

    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                state.running.store(false);
                return false;

            case SDL_KEYDOWN:
                // F11 or Alt+Enter: switch between a window and full screen
                if (event.key.repeat == 0 &&
                    (event.key.keysym.sym == SDLK_F11 ||
                     (event.key.keysym.sym == SDLK_RETURN && (event.key.keysym.mod & KMOD_ALT)))) {
                    if (SDL_Window* w = SDL_GetWindowFromID(event.key.windowID)) {
                        bool full = SDL_GetWindowFlags(w) & SDL_WINDOW_FULLSCREEN_DESKTOP;
                        SDL_SetWindowFullscreen(w, full ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
                    }
                    break;
                }
                handleKeyDown(event.key.keysym.sym, state);
                break;

            case SDL_KEYUP:
                handleKeyUp(event.key.keysym.sym, state);
                break;

            case SDL_MOUSEMOTION:
                state.mouse_x.store(event.motion.x);
                state.mouse_y.store(event.motion.y);
                break;

            case SDL_MOUSEBUTTONDOWN:  // the start screen is worked with the mouse
                if (std::getenv("VEXSIM_TRACE")) fprintf(stderr, "CLICK %d %d button %d\n", event.button.x, event.button.y, event.button.button);
                if (!game().dash && game().matchOver) game().nextRequested = true;  // a click is NEXT after a match
                if (game().dash)
                    for (const DashWidget& d : dashWidgets(game()))
                        if (d.on && d.id >= 0 && d.hit(event.button.x, event.button.y)) {
                            dashClick(game(), d.id, event.button.button == SDL_BUTTON_RIGHT ? -1 : 1);
                            break;
                        }
                break;
        }
    }

    updateAxes(state);
    return true;
}

static void setButton(SimState& state, controller_digital_e_t btn, bool pressed) {
    uint32_t mask = 1u << static_cast<int>(btn);
    if (pressed)
        state.controller.buttons.fetch_or(mask);
    else
        state.controller.buttons.fetch_and(~mask);
}

// Keyboard -> controller buttons, one hand on each side of the keyboard: the left hand drives on WASD, the right hand
// works the arm on the arrow keys (Up / Down lift, Right closes the claw, Left opens it). E / Q / Space / F do the same
// for people who prefer to keep both hands on the left. The robot uses no other button.
static int buttonFor(SDL_Keycode key) {
    switch (key) {
        case SDLK_UP: case SDLK_e:             return E_CONTROLLER_DIGITAL_L1;
        case SDLK_DOWN: case SDLK_q:           return E_CONTROLLER_DIGITAL_L2;
        case SDLK_RIGHT: case SDLK_SPACE:      return E_CONTROLLER_DIGITAL_R1;
        case SDLK_LEFT: case SDLK_f:           return E_CONTROLLER_DIGITAL_R2;
        default:                               return -1;
    }
}

void InputHandler::handleKeyDown(SDL_Keycode key, SimState& state) {
    // After a match: Enter or Space is NEXT (back to the start screen)
    if (!game().dash && game().matchOver && (key == SDLK_RETURN || key == SDLK_SPACE)) { game().nextRequested = true; return; }
    // Start screen: Enter or Space plays; F6 drives without a new match; Esc quits
    // Up / Down pick a setting, Left / Right change it
    if (game().dash) {
        Game& g = game();
        if (g.qCountEdit) {  // typing how many qualification matches to simulate: digits; Enter accepts, Esc cancels
            if (key >= SDLK_0 && key <= SDLK_9 && g.qCountBuf.size() < 2) g.qCountBuf += char('0' + (key - SDLK_0));
            else if (key >= SDLK_KP_1 && key <= SDLK_KP_9 && g.qCountBuf.size() < 2) g.qCountBuf += char('1' + (key - SDLK_KP_1));
            else if (key == SDLK_KP_0 && g.qCountBuf.size() < 2) g.qCountBuf += '0';
            else if (key == SDLK_BACKSPACE && !g.qCountBuf.empty()) g.qCountBuf.pop_back();
            else if ((key == SDLK_RETURN || key == SDLK_KP_ENTER) && !g.qCountBuf.empty()) { g.qCount = std::clamp(std::atoi(g.qCountBuf.c_str()), 1, 99); g.qCountEdit = false; }
            else if (key == SDLK_ESCAPE || key == SDLK_RETURN || key == SDLK_KP_ENTER) g.qCountEdit = false;
            return;
        }
        if (g.timeEdit) {  // typing START TIME: digits and ':' ; Enter accepts, Esc cancels
            if (key >= SDLK_0 && key <= SDLK_9 && g.timeBuf.size() < 5) g.timeBuf += char('0' + (key - SDLK_0));
            else if (key >= SDLK_KP_1 && key <= SDLK_KP_9 && g.timeBuf.size() < 5) g.timeBuf += char('1' + (key - SDLK_KP_1));
            else if (key == SDLK_KP_0 && g.timeBuf.size() < 5) g.timeBuf += '0';
            else if ((key == SDLK_COLON || key == SDLK_SEMICOLON || key == SDLK_PERIOD || key == SDLK_COMMA || key == SDLK_m) && g.timeBuf.find(':') == std::string::npos && !g.timeBuf.empty()) g.timeBuf += ':';
            else if (key == SDLK_BACKSPACE && !g.timeBuf.empty()) g.timeBuf.pop_back();
            else if ((key == SDLK_RETURN || key == SDLK_KP_ENTER) && !g.timeBuf.empty()) {
                g.set.startSecs = parseStartTime(g.timeBuf); g_matchLen = g.set.startSecs; g.matchT = g_matchLen; g.timeEdit = false;
            } else if (key == SDLK_ESCAPE || key == SDLK_RETURN || key == SDLK_KP_ENTER) g.timeEdit = false;
            return;
        }
        if (key == SDLK_RETURN || key == SDLK_SPACE) {
            const auto w = dashWidgets(g);  // Enter presses the highlighted button, or the page's main one
            const int id = (g.dashSel >= 0 && g.dashSel < (int)w.size()) ? w[g.dashSel].id : -1;
            const bool pickBtn = id == D_BRACKET || id == D_SIMONE || (id >= D_BSIM && id <= D_QPAGE);
            if (pickBtn) dashClick(g, id, 1);
            else if (g.dashPage == 1) dashClick(g, D_BSIM, 1);
            else if (g.dashPage == 2) dashClick(g, D_BSTART, 1);
            else if (g.dashPage == 3) dashClick(g, g.previewFast ? D_PSIM : D_PWATCH, 1);
            else if (g.dashPage == 6) dashClick(g, g.qDone >= qualTotal(g) ? D_QALLI : D_QSIM, 1);
            else g.playRequested = true;
            return;
        }
        if (key == SDLK_UP || key == SDLK_DOWN || key == SDLK_w || key == SDLK_s) {
            const auto w = dashWidgets(g);
            const auto pick = dashPickable(w);
            int n = (int)pick.size(), cur = n - 1;
            for (int k = 0; k < n; ++k) if (pick[k] == g.dashSel) cur = k;
            cur = (cur + ((key == SDLK_UP || key == SDLK_w) ? -1 : 1) + n) % n;
            g.dashSel = pick[cur];
            return;
        }
        if (key == SDLK_LEFT || key == SDLK_RIGHT || key == SDLK_a || key == SDLK_d) {
            const int dir = (key == SDLK_LEFT || key == SDLK_a) ? -1 : 1;
            const auto w = dashWidgets(g);
            if (g.dashSel >= 0 && g.dashSel < (int)w.size()) {
                const int id = w[g.dashSel].id;
                if (id >= D_BOTS && id < D_BOTS + BOT_CHOICE_N)
                    dashClick(g, D_BOTS + std::clamp(g.set.bots + dir, 0, BOT_CHOICE_N - 1), 1);
                else if (id != D_PLAY) { g_dashKeyStep = true; dashClick(g, id, dir); g_dashKeyStep = false; }
            }
            return;
        }
        if (key != SDLK_F6 && key != SDLK_ESCAPE) return;
    }
    // Settings menu: Tab opens / closes it; while open, keys move through it and the robot doesn't drive
    Game& gmenu = game();
    if (key == SDLK_TAB) { gmenu.menuOpen = !gmenu.menuOpen; return; }
    if (gmenu.menuOpen) {
        switch (key) {
            case SDLK_UP: case SDLK_w:     gmenu.menuSel = (gmenu.menuSel + MENU_N - 1) % MENU_N; break;
            case SDLK_DOWN: case SDLK_s:   gmenu.menuSel = (gmenu.menuSel + 1) % MENU_N; break;
            case SDLK_LEFT: case SDLK_a:   menuChange(gmenu.menuSel, -1); break;
            case SDLK_RIGHT: case SDLK_d:
            case SDLK_RETURN: case SDLK_SPACE: menuChange(gmenu.menuSel, +1); break;
            case SDLK_ESCAPE:              gmenu.menuOpen = false; break;
        }
        return;
    }
    if (int b = buttonFor(key); b >= 0) { if (game().set.pilot == 0) setButton(state, (controller_digital_e_t)b, true); return; }
    switch (key) {
        // Drive keys
        case SDLK_w:      key_w_     = true; break;
        case SDLK_s:      key_s_     = true; break;
        case SDLK_a:      key_a_     = true; break;
        case SDLK_d:      key_d_     = true; break;
        case SDLK_UP:     key_up_    = true; break;
        case SDLK_DOWN:   key_down_  = true; break;
        case SDLK_LEFT:   key_left_  = true; break;
        case SDLK_RIGHT:  key_right_ = true; break;

        // Competition mode — set abort_task first so the current mode fn unwinds
        case SDLK_F6:
            game().dash = false;
            state.abort_task.store(true);
            state.comp_mode.store(CompMode::OPCONTROL);
            break;
        case SDLK_F8: game().endRequested = true; break;  // end the match now
        case SDLK_F9: game().skipTo = 15.0; break;        // jump the clock to 15 s left
        case SDLK_F7:
            state.abort_task.store(true);
            state.comp_mode.store(CompMode::DISABLED);
            break;


        // 1 to 9 and 0: pick a 3D camera directly (C / V step through them)
        case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4: case SDLK_5:
        case SDLK_6: case SDLK_7: case SDLK_8: case SDLK_9:
            game().camMode = (key - SDLK_1) % CAM_COUNT;
            break;
        case SDLK_0: game().camMode = 9 % CAM_COUNT; break;

        // Z / X: drive team drops a match load (a pin / a cup) into the nearest red Loader
        case SDLK_z: game().loadRequest = 1; break;
        case SDLK_x: game().loadRequest = 2; break;

        // R: put the field and robot back to the match start
        case SDLK_r:
            game().resetRequested = true;
            break;
        // C / V: next / previous 3D camera (see camName in Game.hpp)
        case SDLK_c:
            game().camMode = (game().camMode + 1) % CAM_COUNT;
            break;
        case SDLK_v:
            game().camMode = (game().camMode + CAM_COUNT - 1) % CAM_COUNT;
            break;

        case SDLK_ESCAPE:
            state.running.store(false);
            break;
    }
}

void InputHandler::handleKeyUp(SDL_Keycode key, SimState& state) {
    if (int b = buttonFor(key); b >= 0) { if (game().set.pilot == 0) setButton(state, (controller_digital_e_t)b, false); return; }
    switch (key) {
        case SDLK_w:     key_w_     = false; break;
        case SDLK_s:     key_s_     = false; break;
        case SDLK_a:     key_a_     = false; break;
        case SDLK_d:     key_d_     = false; break;
        case SDLK_UP:    key_up_    = false; break;
        case SDLK_DOWN:  key_down_  = false; break;
        case SDLK_LEFT:  key_left_  = false; break;
        case SDLK_RIGHT: key_right_ = false; break;
    }
}

// Arcade driving on WASD: W forward, S back, A turn left, D turn right. Sent as a split
// arcade controller: left stick Y = forward, right stick X = turn. The robot code does the mixing.
void InputHandler::updateAxes(SimState& state) {
    if (game().set.pilot > 0) return;  // YOUR ROBOT is driven by the CPU (Game.hpp pilotUpdate)
    if (game().menuOpen) {  // no driving while the settings menu is open
        state.controller.leftY.store(0); state.controller.rightY.store(0);
        state.controller.leftX.store(0); state.controller.rightX.store(0);
        return;
    }
    const double ARCADE_TURN = game().set.turnPct / 100.0;  // how far a turn key pushes the right stick (settings menu)
    int fwd = ((key_w_ || key_up_) ? 127 : 0) - ((key_s_ || key_down_) ? 127 : 0);
    int turn = (int)((((key_d_ || key_right_) ? 127 : 0) - ((key_a_ || key_left_) ? 127 : 0)) * ARCADE_TURN);
    state.controller.leftY.store(fwd);
    state.controller.rightY.store(0);
    state.controller.leftX.store(0);
    state.controller.rightX.store(std::clamp(turn, -127, 127));
}

} // namespace sim
