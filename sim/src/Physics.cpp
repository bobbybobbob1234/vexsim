#define _USE_MATH_DEFINES
#include "sim/Physics.hpp"
#include "sim/SimState.hpp"
#include "sim/Game.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>

namespace sim {

Physics::Physics(const RobotConfig& cfg) : cfg_(cfg) {
    // Goal positions in field inches (origin = bottom-left, Y up).
    // Derived from 2026-27 Appendix A diagram (field = 140.41" × 140.41").
    // Collision radius: half of short-goal base width (5.61") = 2.81"
    const double S  = cfg_.field_w / 140.41; // px per inch
    const double FH = cfg_.field_h;
    const double R  = 3.22 * S;              // collision radius = outer octagonal circumradius (px)

    struct { double xi, yi; } pos[9] = {
        // Red alliance goals (bottom-left quadrant)
        {23.11,  46.66}, {46.66, 23.11},
        // Blue alliance goals (top-right quadrant)
        {117.30, 93.75}, {93.75, 117.30},
        // Neutral goals (top-left and bottom-right quadrants)
        {46.66, 117.30}, {23.11, 93.75},
        {117.30, 46.66}, {93.75, 23.11},
        // Center tall goal
        {70.20, 70.20},
    };
    for (int i = 0; i < 9; ++i)
        goals_[i] = { pos[i].xi * S, FH - pos[i].yi * S, R };
    for (int i = 0; i < 9; ++i) solids_[i] = goals_[i];
    for (int i = 0; i < 4; ++i)  // Loaders: about 4 x 3.74 in on the wall, as a circle
        solids_[9 + i] = { LOADERS[i].x * S, FH - LOADERS[i].y * S, 2.1 * S };
}

double Physics::averageEffectiveVoltage(const std::array<int, 4>& ports, double speed_frac) const {
    double sum = 0.0;
    int count  = 0;
    for (int port : ports) {
        if (port == 0) break;
        int idx = std::abs(port);
        sum += SimState::get().motors[idx].get_effective_voltage(speed_frac, cfg_.gear_friction_coeff);
        ++count;
    }
    return count > 0 ? sum / count : 0.0;
}

bool Physics::allCoast(const std::array<int, 4>& ports) const {
    for (int port : ports) {
        if (port == 0) break;
        if (SimState::get().motors[std::abs(port)].brake_mode_int.load() >= 1) return false;
    }
    return true;
}

void Physics::step(double dt_sec) {
    auto& state = SimState::get();
    if (game().playerOff) {  // the player's robot sits this match out: parked far off the field
        {
            std::lock_guard<std::mutex> lock(state.pose_mutex);
            state.pose.x = state.pose.y = -4000.0;
        }
        vL_ = vR_ = vxw_ = vyw_ = 0.0;
        gameUpdate(state, cfg_, goals_, dt_sec, 0.0);
        static double offCarry = 0.0;
        offCarry += dt_sec * 1000.0;
        const uint32_t whole = static_cast<uint32_t>(offCarry + 1e-6);
        offCarry -= whole;
        state.sim_millis.fetch_add(whole);
        return;
    }

    // Max linear wheel speed in pixels/sec.
    // gear_ratio = wheel_rpm / motor_rpm, so effective wheel RPM = max_rpm * gear_ratio.
    double wheel_circ = 2.0 * M_PI * cfg_.wheel_radius_px;
    double max_speed  = (g_wheelRpm / 60.0) * wheel_circ;  // the DRIVE RPM setting

    // Motors report their own effective voltage:
    //   - Powered:     commanded voltage (normal drive)
    //   - BRAKE/HOLD + unpowered: back-EMF opposing current motion (motor generates this)
    //   - COAST + unpowered:      0 (motor disconnected)
    double eff_vL = averageEffectiveVoltage(cfg_.left_ports,  vL_ / max_speed);
    double eff_vR = averageEffectiveVoltage(cfg_.right_ports, vR_ / max_speed);

    double target_vL = (eff_vL / 127.0) * max_speed;
    double target_vR = (eff_vR / 127.0) * max_speed;

    // COAST uses a long friction-only tau; powered/braking uses the motor tau.
    double tauL = (std::abs(eff_vL) < 1e-3 && allCoast(cfg_.left_ports))
                  ? cfg_.coast_time_constant : cfg_.accel_time_constant / PLAYER_TORQUE;  // more torque: the wheels catch up sooner
    double tauR = (std::abs(eff_vR) < 1e-3 && allCoast(cfg_.right_ports))
                  ? cfg_.coast_time_constant : cfg_.accel_time_constant / PLAYER_TORQUE;  // more torque: the wheels catch up sooner

    // Motor lag, then a traction limit: the wheels slip past about 0.8 g of acceleration. With the
    // lift up the robot is top-heavy: a driver (and the wheels) can only use about half of that
    // before it tips, so acceleration and braking soften as the lift rises.
    const double liftFrac = std::clamp((game().liftIn - LIFT_BASE_IN) / (dr4bMaxLiftIn() - LIFT_BASE_IN), 0.0, 1.0);
    Game& gmT = game();
    const double hlIn = cfg_.robot_half_w / (720.0 / 140.41);
    robotMassProps(gmT, hlIn);  // centre of mass for this step (lift height, held pieces)
    double a_max = 0.8 * 386.1 * (720.0 / 140.41);  // px/s^2
    if (!gmT.set.tipping) {
        a_max *= 1.0 - 0.55 * liftFrac;  // tipping off: the old stand-in for a top-heavy robot
    } else {
        // Leaning over: the wheels on the high side lift off and the drive loses grip
        double lean = std::max(std::fabs(gmT.pitch), std::fabs(gmT.roll));
        // (a small lean, such as bracing against a wall or goal, costs nothing: the drive only loses grip once a wheel is really coming up)
        a_max *= std::max(0.2, 1.0 - std::max(0.0, lean - 0.15) / 0.35);
    }
    if (gmT.tipped) {  // on its side: the wheels spin in the air
        target_vL = target_vR = 0.0;
        vL_ = vR_ = 0.0;
        vxw_ = vyw_ = 0.0;
    }
    double hitCut = 0.0, hitZ = 2.5;  // biggest collision this step and the height it hit at (in)
    auto noteHit = [&](double cut, double z) {
        if (std::fabs(cut) > hitCut) { hitCut = std::fabs(cut); hitZ = z; }
    };
    double dvL = (target_vL - vL_) * std::min(1.0, dt_sec / tauL);
    double dvR = (target_vR - vR_) * std::min(1.0, dt_sec / tauR);
    // drive torque (4 motors geared to the DRIVE RPM): the force falls as each side nears top speed
    const double aL = std::min(a_max, driveAccelLimit(g_wheelRpm, vL_, max_speed, dvL)), aR = std::min(a_max, driveAccelLimit(g_wheelRpm, vR_, max_speed, dvR));
    vL_ += std::clamp(dvL, -aL * dt_sec, aL * dt_sec);
    vR_ += std::clamp(dvR, -aR * dt_sec, aR * dt_sec);
    double impact = 0.0;  // in/s of forward speed lost to a wall or goal this step
    double driveVx = 0.0, driveVy = 0.0;  // robot velocity from the wheels, before any collision

    double vL = vL_;
    double vR = vR_;

    // Differential drive kinematics
    double v     = (vL + vR) / 2.0;                       // linear velocity px/s
    // Screen y points down, so a positive heading change turns the robot clockwise on screen:
    // left side faster than right = clockwise (a right turn), like the real robot
    double omega = (vL - vR) / cfg_.track_width_px;       // angular velocity rad/s

    // Wheel spin for drawing (radians), from each side's surface speed
    game().wheelRotL += vL / cfg_.wheel_radius_px * dt_sec;
    game().wheelRotR += vR / cfg_.wheel_radius_px * dt_sec;

    {
        std::lock_guard<std::mutex> lock(state.pose_mutex);
        state.pose.heading += omega * dt_sec;
        state.pose.heading_accumulated += omega * dt_sec;

        // Drift: the wheels set the speed along the robot's heading. The front and back omni wheels
        // hold it sideways only weakly; the middle traction wheels (about a third of the weight) grip. Whatever sideways speed the robot carries (left over from its
        // old heading when it turns) bleeds off at most DRIFT_GRIP_G, so a hard turn at speed
        // slides out a little before the robot follows its new heading.
        {
            const double DRIFT_GRIP_G = game().set.omniBot[3] ? 0.35 : 0.6;  // sideways grip, in g: omni rollers about 0.35 on two thirds of the weight, rubber traction tread about 1.1 on the middle third
            const double aLat = DRIFT_GRIP_G * 386.1 * (720.0 / 140.41);  // px/s^2
            double dxh = std::cos(state.pose.heading), dyh = std::sin(state.pose.heading);
            double nxh = -dyh, nyh = dxh;
            double vPerp = game().set.drift ? vxw_ * nxh + vyw_ * nyh : 0.0;  // drifting can be switched off
            double bleed = std::min(std::fabs(vPerp), aLat * dt_sec);
            vPerp -= vPerp > 0 ? bleed : -bleed;
            vxw_ = dxh * v + nxh * vPerp;
            vyw_ = dyh * v + nyh * vPerp;
            if (gmT.tipped) vxw_ = vyw_ = 0.0;
        }
        driveVx = vxw_;
        driveVy = vyw_;
        state.pose.x += vxw_ * dt_sec;
        state.pose.y += vyw_ * dt_sec;
        poseAfterMoveX_ = state.pose.x;
        poseAfterMoveY_ = state.pose.y;

        // Goal collision: circle-circle push-out.
        // Robot is approximated as a circle (conservative circumscribed radius).
        // The robot is a rotated rectangle; each goal is a circle.
        const double hw = cfg_.robot_half_w;
        const double hh = cfg_.robot_half_h;
        for (const auto& g : solids_) {
            double c = std::cos(state.pose.heading), s = std::sin(state.pose.heading);
            double dx = g.x - state.pose.x;
            double dy = g.y - state.pose.y;
            double lx = dx * c + dy * s;     // goal center in robot space (+x = front)
            double ly = -dx * s + dy * c;
            // A goal lined up with the front opening drives into it (the alignment guide handles it)
            if (std::fabs(ly) < OPEN_HALF_IN * (720.0 / 140.41) && lx > hw - OPEN_DEPTH_IN * (720.0 / 140.41)) continue;
            double qx = std::clamp(lx, -hw, hw);
            double qy = std::clamp(ly, -hh, hh);
            double ex = lx - qx, ey = ly - qy;
            double dist = std::sqrt(ex * ex + ey * ey);
            if (dist >= g.r) continue;

            // Unit vector from the robot toward the goal, in field space
            double nlx, nly;
            if (dist > 1e-9) { nlx = ex / dist; nly = ey / dist; }
            else             { double d = std::sqrt(lx * lx + ly * ly); nlx = d > 1e-9 ? lx / d : 1.0; nly = d > 1e-9 ? ly / d : 0.0; }
            double nx = nlx * c - nly * s;
            double ny = nlx * s + nly * c;

            // Push the robot back out of the goal
            double push = g.r - dist;
            state.pose.x -= nx * push;
            state.pose.y -= ny * push;

            // Stop the forward/backward motion into the goal but keep any turning
            double v_lin = (vL_ + vR_) * 0.5;
            if (v_lin * c * nx + v_lin * s * ny > 0.0) {
                impact = std::max(impact, std::fabs(v_lin) / (720.0 / 140.41));
                noteHit(v_lin, 2.5);  // goal or Loader against the chassis rails
                vL_ -= v_lin;
                vR_ -= v_lin;
            }
        }

        // Goal alignment guide (the V on the front): its arms and back push the robot off a goal's
        // base, and because the arms are angled that push is partly sideways, centering the goal
        {
            const double S = 720.0 / 140.41;
            struct Seg { double ax, ay, bx, by; };
            const Seg segs[3] = {
                {hw + ALIGN_BACK_X * S, -ALIGN_BACK_HALF * S, hw + ALIGN_MOUTH_X * S, -ALIGN_MOUTH_HALF * S},
                {hw + ALIGN_BACK_X * S,  ALIGN_BACK_HALF * S, hw + ALIGN_MOUTH_X * S,  ALIGN_MOUTH_HALF * S},
                {hw + ALIGN_BACK_X * S, -ALIGN_BACK_HALF * S, hw + ALIGN_BACK_X * S,   ALIGN_BACK_HALF * S}};
            for (int gi = 0; gi < 9; ++gi) {
                const auto& g = goals_[gi];
                double c = std::cos(state.pose.heading), s = std::sin(state.pose.heading);
                double dx = g.x - state.pose.x, dy = g.y - state.pose.y;
                double lx = dx * c + dy * s, ly = -dx * s + dy * c;  // goal center, robot frame
                const double r = ALIGN_GOAL_R * S;
                for (const Seg& sg : segs) {
                    double ex = sg.bx - sg.ax, ey = sg.by - sg.ay;
                    double t = std::clamp(((lx - sg.ax) * ex + (ly - sg.ay) * ey) / (ex * ex + ey * ey), 0.0, 1.0);
                    double qx = sg.ax + ex * t, qy = sg.ay + ey * t;
                    double ox = qx - lx, oy = qy - ly, d = std::hypot(ox, oy);
                    if (d >= r || d < 1e-9) continue;
                    // Push the guide (and robot) away from the goal, in robot frame then field frame
                    double pushL = r - d, ux = ox / d, uy = oy / d;
                    double wxp = (ux * c - uy * s) * pushL, wyp = (ux * s + uy * c) * pushL;
                    state.pose.x += wxp;
                    state.pose.y += wyp;
                    lx -= ux * pushL; ly -= uy * pushL;
                    double v_lin = (vL_ + vR_) * 0.5;
                    if (v_lin * ux < 0.0) {  // the guide was moving into the goal: stop the forward part
                        double cut = v_lin * std::fabs(ux);
                        noteHit(cut, (ALIGN_Z0 + ALIGN_Z1) / 2);  // the aligner meets the goal base low down
                        vL_ -= cut;
                        vR_ -= cut;
                    }
                }
            }
        }

        // Clamp to field bounds using the rotated robot's extent; driving into a wall stops forward motion
        {
            double c = std::fabs(std::cos(state.pose.heading)), s = std::fabs(std::sin(state.pose.heading));
            double ex = c * hw + s * hh;
            double ey = s * hw + c * hh;
            double nx = std::clamp(state.pose.x, ex, cfg_.field_w - ex);
            double ny = std::clamp(state.pose.y, ey, cfg_.field_h - ey);
            if (nx != state.pose.x || ny != state.pose.y) {
                double v_lin = (vL_ + vR_) * 0.5;
                double wx = (nx - state.pose.x), wy = (ny - state.pose.y);  // points away from the wall
                double vx = v_lin * std::cos(state.pose.heading), vy = v_lin * std::sin(state.pose.heading);
                if (vx * wx + vy * wy < 0.0) {
                    impact = std::max(impact, std::fabs(v_lin) / (720.0 / 140.41));
                    noteHit(v_lin, 2.5);  // the wall meets the bumper rails
                    vL_ -= v_lin;
                    vR_ -= v_lin;
                }
                state.pose.x = nx;
                state.pose.y = ny;
            }
        }

        // The toggle arms on the back of the DR6B. Their pads ride up and down with the lift:
        //  - below the 11.5 in wall panels a pad can't go through the wall, so it stops the robot short;
        //  - at Toggle height it stops against a Toggle (fixed on the wall mounts), which it rolls
        //    when it hits one hard enough;
        //  - above the Toggles it passes over the wall freely.
        {
            const double S = 720.0 / 140.41, F = 140.41;
            double c = std::cos(state.pose.heading), s = std::sin(state.pose.heading);
            double padX, padZ;
            ramPadPose(gmT.liftAngle, cfg_.robot_half_w / S, padX, padZ);
            const double tipB = padX * S;  // negative: behind the robot's centre
            for (double sd : {-1.0, 1.0}) {
                double wx = state.pose.x + c * tipB - s * sd * RAM_Y_IN * S;
                double wy = state.pose.y + s * tipB + c * sd * RAM_Y_IN * S;
                if (padZ < RAM_Z_LO) {
                    double fwd = 0.0;  // how far to move the robot forward to bring the pad back inside the walls
                    if (wx < 0 && c > 1e-3) fwd = std::max(fwd, -wx / c);
                    if (wx > cfg_.field_w && c < -1e-3) fwd = std::max(fwd, (wx - cfg_.field_w) / -c);
                    if (wy < 0 && s > 1e-3) fwd = std::max(fwd, -wy / s);
                    if (wy > cfg_.field_h && s < -1e-3) fwd = std::max(fwd, (wy - cfg_.field_h) / -s);
                    if (fwd > 0) {
                        state.pose.x += c * fwd;
                        state.pose.y += s * fwd;
                        double v_lin = (vL_ + vR_) * 0.5;
                        if (v_lin < 0) { noteHit(v_lin, padZ); vL_ -= v_lin; vR_ -= v_lin; }
                    }
                    continue;
                }
                if (padZ > RAM_Z_HI) continue;
                double fx = wx / S, fy = (720.0 - wy) / S;
                for (int k = 0; k < 4; ++k) {
                    double across = k == 0 ? fx - TOGGLE_INSET_IN : k == 1 ? F - TOGGLE_INSET_IN - fx
                                  : k == 2 ? fy - TOGGLE_INSET_IN : F - TOGGLE_INSET_IN - fy;
                    double along = (k <= 1) ? fy - F / 2 : fx - F / 2;
                    if (std::fabs(along) >= 13.0 || across >= TOGGLE_R_IN + 0.3) continue;
                    // Push the robot straight forward along its heading, out of the Toggle
                    double into = (TOGGLE_R_IN + 0.3 - across) * S;
                    double hx = k == 0 ? -1 : k == 1 ? 1 : 0, hy = k == 2 ? 1 : k == 3 ? -1 : 0;  // wall direction, sim px
                    double toward = -(c * hx + s * hy);  // how squarely the robot's back faces that wall
                    if (toward <= 0.05) continue;
                    double fwdPush = into / toward;
                    state.pose.x += c * fwdPush;
                    state.pose.y += s * fwdPush;
                    double v_lin = (vL_ + vR_) * 0.5;
                    if (v_lin < 0) { noteHit(v_lin, padZ); vL_ -= v_lin; vR_ -= v_lin; }  // high up: pushes the top forward
                }
            }
        }

        // The claw sticks out past the front bumper: its jaw tips can't go through the walls, or
        // into a goal while they are below its top
        {
            const auto& gm = game();
            double c = std::cos(state.pose.heading), s = std::sin(state.pose.heading);
            double reachPx = hw + JAW_LEN_IN * (720.0 / 140.41), sidePx = 1.5 * (720.0 / 140.41);
            for (double sd : {-1.0, 0.0, 1.0}) {
                double tx = state.pose.x + c * reachPx - s * sidePx * sd;
                double ty = state.pose.y + s * reachPx + c * sidePx * sd;
                double back = 0.0;
                // Walls: how far the tip is past one, measured along the robot's heading
                if (tx < 0 && c < -1e-3) back = std::max(back, -tx / -c);
                if (tx > cfg_.field_w && c > 1e-3) back = std::max(back, (tx - cfg_.field_w) / c);
                if (ty < 0 && s < -1e-3) back = std::max(back, -ty / -s);
                if (ty > cfg_.field_h && s > 1e-3) back = std::max(back, (ty - cfg_.field_h) / s);
                for (int i = 0; i < 9; ++i) {
                    if (gm.liftIn + 2.0 >= GOAL_H_IN[i] - 1.5) continue;  // jaws high enough to reach over it
                    const auto& g = goals_[i];
                    double d = std::hypot(tx - g.x, ty - g.y);
                    if (d < g.r * 0.85) back = std::max(back, g.r * 0.85 - d);
                }
                if (back > 0) {
                    state.pose.x -= c * back;
                    state.pose.y -= s * back;
                    double v_lin = (vL_ + vR_) * 0.5;
                    if (v_lin > 0) {
                        impact = std::max(impact, v_lin / (720.0 / 140.41));
                        noteHit(v_lin, gm.liftIn + 1.5);  // the claw's wheel stack
                        vL_ -= v_lin;
                        vR_ -= v_lin;
                    }
                }
            }
        }
    }

    // Anything that pushed the robot this step also stops its sliding into it
    {
        std::lock_guard<std::mutex> lock(state.pose_mutex);
        double px = state.pose.x - poseAfterMoveX_, py = state.pose.y - poseAfterMoveY_;
        double pl = std::hypot(px, py);
        if (pl > 1e-6) {
            double ux = px / pl, uy = py / pl;      // direction the robot was pushed
            double into = vxw_ * ux + vyw_ * uy;    // negative = moving into the obstacle
            if (into < 0) { vxw_ -= ux * into; vyw_ -= uy * into; }
        }
    }

    // Tipping: the chassis' acceleration from the wheels (including the centripetal part in a turn)
    // and the sudden speed change from any collision, both in the robot's frame, in inches
    if (dt_sec > 0) {
        const double S = 720.0 / 140.41;
        double hd;
        {
            std::lock_guard<std::mutex> lock(state.pose_mutex);
            hd = state.pose.heading;
        }
        const double c = std::cos(hd), s = std::sin(hd);  // forward (sim px, y down); left is (s, -c)
        double ax = (driveVx - endVx_) / dt_sec, ay = (driveVy - endVy_) / dt_sec;
        double dx = vxw_ - driveVx, dy = vyw_ - driveVy;
        // Which sides are up against something (a wall, goal or Loader within 0.8 in, or the ram
        // pads on a Toggle): the robot can only lean that way until it rests on it
        bool brace[4] = {false, false, false, false};  // front, back, left, right
        {
            double px, py;
            {
                std::lock_guard<std::mutex> lock(state.pose_mutex);
                px = state.pose.x; py = state.pose.y;
            }
            const double hw = cfg_.robot_half_w, hh = cfg_.robot_half_h, gap = 0.8 * S;
            auto blocked = [&](double lx, double ly) {  // robot-frame px: x forward, y left
                double wx = px + c * lx + s * ly, wy = py + s * lx - c * ly;
                if (wx < 0 || wy < 0 || wx > cfg_.field_w || wy > cfg_.field_h) return true;
                for (const auto& o : solids_)
                    if (std::hypot(wx - o.x, wy - o.y) < o.r) return true;
                return false;
            };
            for (double f : {-0.9, 0.0, 0.9}) {
                if (blocked(hw + gap, f * hh))  brace[0] = true;
                if (blocked(-hw - gap, f * hh)) brace[1] = true;
                if (blocked(f * hw, hh + gap))  brace[2] = true;
                if (blocked(f * hw, -hh - gap)) brace[3] = true;
            }
            // Ram pads on a Toggle: leaning either way slides them up its rounded face and off,
            // so the lean stays small
            for (bool t : gmT.touching) if (t) brace[0] = brace[1] = true;
        }
        robotTipStep(gmT, hlIn, (ax * c + ay * s) / S, (ax * s - ay * c) / S,
                     (dx * c + dy * s) / S, (dx * s - dy * c) / S, hitZ, dt_sec, brace);
    }

    // Non-drive motors (lift, claw, intake...) turn at their commanded speed, so
    // get_position() changes. Green-cartridge scale: 127 = 200 rpm.
    for (int port = 1; port < MAX_PORTS; ++port) {
        bool drive = port == cfg_.tracking_left_port || port == cfg_.tracking_right_port ||
                     port == cfg_.tracking_mid_port;
        for (int p : cfg_.left_ports)  if (std::abs(p) == port) drive = true;
        for (int p : cfg_.right_ports) if (std::abs(p) == port) drive = true;
        if (drive) continue;
        auto& m = state.motors[port];
        bool idle = m.voltage.load() == 0 && m.millivolts.load() == 0 && m.target_velocity.load() == 0;
        double target;
        if (idle)                                        target = 0.0;
        else if (m.mode == MotorState::Mode::VELOCITY)   target = m.target_velocity.load();
        else if (m.mode == MotorState::Mode::MILLIVOLTS) target = m.millivolts.load() / 12000.0 * 200.0;
        else                                             target = m.voltage.load() / 127.0 * 200.0;

        bool isLift = port == LIFT_PORT || port == LIFT_PORT2;
        if (isLift) {
            if (idle) {
                // Nothing driving it: HOLD keeps it there, BRAKE lets it creep down, COAST (or a
                // disabled robot) lets it fall
                int bm = state.comp_mode.load() == CompMode::DISABLED ? 0 : m.brake_mode_int.load();
                target = bm == 2 ? 0.0 : (bm == 1 ? -2.0 : -45.0);
            } else if (target > 0) {
                target *= 0.93 - (game().held >= 0 ? 0.06 : 0.0) - (game().rider >= 0 ? 0.05 : 0.0);  // lifting against gravity (and a load) is slower
            }
        }
        if (isLift && !idle) target *= g_liftSpeed;  // lift type: a cascade is the fastest
        // Ratcheting claw: an open command first lifts the pawl (the jaws can't open until it is
        // clear of the teeth); any other command lets it drop back onto the ratchet wheel
        bool isClaw = port == CLAW_PORT;
        if (isClaw) {
            auto& gm = game();
            if (target < 0) gm.pawlUp = std::min(1.0, gm.pawlUp + dt_sec / PAWL_LIFT_S);
            else            gm.pawlUp = std::max(0.0, gm.pawlUp - dt_sec / PAWL_LIFT_S);
            if (target < 0 && gm.pawlUp < 1.0) target = 0.0;
            gm.pawlClick = std::max(0.0, gm.pawlClick - dt_sec / 0.06);
        }

        double& rpm = mechRpm_[port];
        rpm += (target - rpm) * std::min(1.0, dt_sec / 0.08);  // motor and mechanism lag
        if (isClaw && rpm < 0 && game().pawlUp < 1.0) rpm = 0.0;  // the pawl blocks opening
        if (isClaw) game().clawRpm = rpm;  // the wheel stacks spin in or out for as long as the button is held
        if (std::fabs(rpm) < 1e-3) { rpm = 0.0; continue; }

        double prevPos = m.get_position_deg();
        double pos = prevPos + rpm * 6.0 * dt_sec;
        double lo = 0.0, hi = 1e12;
        if (isLift) hi = LIFT_MAX_DEG;
        if (isLift) {  // the lift has hard stops; the claw's wheels turn without limit
            if (pos <= lo) { pos = lo; if (rpm < 0) rpm = 0; }
            if (pos >= hi) { pos = hi; if (rpm > 0) rpm = 0; }
        }
        if (isClaw && std::floor(pos / RATCHET_TOOTH_DEG) > std::floor(prevPos / RATCHET_TOOTH_DEG))
            game().pawlClick = 1.0;  // the pawl drops over the next tooth
        m.set_position_deg(pos);
    }

    GameResult gr = gameUpdate(state, cfg_, goals_, dt_sec, impact);
    if (gr.robotMoved) {
        vL_ = 0.0;
        vR_ = 0.0;
        vxw_ = vyw_ = 0.0;
    }
    if (gr.pushed > 0) {
        // Shoving pieces along the floor costs speed
        double keep = std::max(0.0, 1.0 - 0.03 * std::min(gr.pushed, 4) * dt_sec);  // pieces weigh almost nothing: barely any drag
        vL_ *= keep;
        vR_ *= keep;
    }
    endVx_ = vxw_;
    endVy_ = vyw_;

    // Update motor encoder positions.
    // (they use the wheel speed left after this step's collisions: a wheel blocked by a goal or the wall has stalled)
    // The motor shaft turns at wheel_speed / gear_ratio — a torque-geared motor
    // (gear_ratio < 1) spins faster than the wheel, so encoders advance more per inch.
    // Negative port → reversed motor → encoder sign is flipped (shaft rotates opposite to wheel).
    for (int port : cfg_.left_ports) {
        if (port == 0) break;
        double deg_per_sec = (vL_ / wheel_circ) * 360.0 / cfg_.gear_ratio;
        state.motors[std::abs(port)].add_position_deg((port < 0 ? -1.0 : 1.0) * deg_per_sec * dt_sec);
    }
    for (int port : cfg_.right_ports) {
        if (port == 0) break;
        double deg_per_sec = (vR_ / wheel_circ) * 360.0 / cfg_.gear_ratio;
        state.motors[std::abs(port)].add_position_deg((port < 0 ? -1.0 : 1.0) * deg_per_sec * dt_sec);
    }

    // Update dedicated tracking wheel rotation sensors (non-motor ports).
    // These use their own wheel radius, which may differ from the drive wheels.
    double tw_circ = 2.0 * M_PI * cfg_.tracking_wheel_radius_px;
    if (cfg_.tracking_left_port > 0) {
        double deg_per_sec = (vL_ / tw_circ) * 360.0;
        state.motors[cfg_.tracking_left_port].add_position_deg(deg_per_sec * dt_sec);
    }
    if (cfg_.tracking_right_port > 0) {
        double deg_per_sec = (vR_ / tw_circ) * 360.0;
        state.motors[cfg_.tracking_right_port].add_position_deg(deg_per_sec * dt_sec);
    }
    // Middle (strafe) tracking wheel: in a diff-drive model, lateral velocity = 0.
    // The port is updated but will always accumulate 0, which is correct —
    // LemLib uses it to detect sideways slip, and an ideal sim has none.
    // (Leave tracking_mid_port at 0 for diff-drive robots.)

    // Advance simulation clock
    // Keep the leftover fraction of a millisecond: truncating each frame's 15.999 ms to 15 made the
    // sim clock run about 6 percent slow at 60 frames per second
    static double msCarry = 0.0;
    msCarry += dt_sec * 1000.0;
    const uint32_t wholeMs = static_cast<uint32_t>(msCarry + 1e-6);
    msCarry -= wholeMs;
    state.sim_millis.fetch_add(wholeMs);
}

} // namespace sim
