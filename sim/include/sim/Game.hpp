#pragma once
// Override (V5RC 2026-27) game layer: pins, cups, a lift with height, a claw whose jaws
// really open and close, goal stacks, scoring by visible pin halves, toggles and the Midfield.
// Piece positions come from the manual's scoring-object diagram and are approximate.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <string>
#include <vector>
#include "SimState.hpp"
#include "Physics.hpp"

namespace sim {

// ── Robot mechanism (this robot: lift on 12 and 19, claw on 7) ────────────────
constexpr int    LIFT_PORT       = 12;
constexpr int    LIFT_PORT2      = 19;
constexpr int    CLAW_PORT       = 7;
constexpr double LIFT_BASE_IN    = 0.5;    // bottom of a held piece with the lift down
constexpr double ROBOT_START_H_IN = 18.0; // tower tops: the robot's height at the start (18 in limit)
// Drive wheels on each side, inches ahead of the robot's centre: one near the front, and two close
// together at the back (3.6 in apart, just clear of each other's 3.25 in tires and 48-tooth gears).
// The front one is as far forward as its motor allows (the motor sits up and ahead of the wheel, and
// its 36-tooth pinion just clears the front cross piece at 8.5 in).
constexpr double WHEEL_X_IN[3] = {6.1, -2.5, -6.1};
constexpr int    TRACTION_WHEEL = 1;  // the middle wheel on each side is a traction wheel; the front and back ones are omni

// Passive toggle ram: two rubber-tipped arms at the front corners of the robot, at the height of
// the field Toggles, each on its own post above the outer drive rail (outside the lift's path). No
// motor: drive the front of the robot into a Toggle and the arms hit it and roll it over exactly
// one face (yellow -> red -> blue -> yellow). Back off before ramming it again.
constexpr double RAM_Y_IN        = 4.3;    // arms this far either side of the centre line (on the lift's middle tower)
constexpr double RAM_ARM_RISE_IN = 3.18;   // a riser on the middle tower lifts each arm this far, so with the lift fully down the pads sit at 13.2 in, the middle of the Toggle bar (they leave Toggle height above about 75 lift motor degrees)
constexpr double RAM_TALL_RISE_IN = 10.7;  // cascade and clawbot: riser height, so with the lift down (claw mount at 2.5 in) the pads sit at 13.2 in
constexpr double RAM_ARM_BACK_IN = 2.7;    // each arm sticks out this far behind the lift's middle tower (folded, its pad is level with the back bumper)
constexpr double RAM_Z_LO        = 11.6;   // pad heights that reach a Toggle (above the 11.5 in wall panel,
constexpr double RAM_Z_HI        = 15.3;   //  up to the top of the Toggle bar plus the pad); lower, it hits the wall
constexpr double RAM_MIN_IPS     = 6.0;    // slowest ram (toward the Toggle) that still rolls it
constexpr double TOGGLE_R_IN     = 1.19;   // Toggle corner radius (2.03 in tall triangle)
constexpr double TOGGLE_INSET_IN = 0.9;    // Toggle pivot line, this far in from the wall's inner face

// Double reverse six-bar (DR6B): two six-bar stages. A six-bar is a 4-bar whose long bottom bar
// runs on past the joint and becomes the "tower" for a second 4-bar that carries on the same way,
// so each stage is two sections pointing the same direction (no gears inside a stage). The two
// stages are tied by one pair of meshing gears in the middle, which makes the second stage swing
// the opposite way: sections go back, back, then forward, forward, and the carriage goes straight
// up. It gains twice the height per degree of a DR4B with the same bars.
// Height = sections x bar length x (sin of bar angle - sin of the lowest angle).
constexpr int    DR4B_SECTIONS   = 4;      // parallelogram sections (2 per six-bar stage, 2 stages)
constexpr double DR4B_BAR_IN     = 7.2;    // pivot-to-pivot length of each bar (a stage folds back 2 bars, inside the robot)
constexpr double DR4B_SPACING_IN = 2.0;    // vertical gap between the two bars of a section
constexpr double DR4B_MIN_DEG    = -41.4;  // bar angle all the way down: folded, the lift's top pivots (plus the bar) are at 18 in
constexpr double DR4B_MAX_DEG    = 57.0;   // bar angle at the top hard stop (claw about 43.7 in, robot top under 50 in)
constexpr double DR4B_GEAR       = 7.0;    // lift motor degrees per bar degree (1:7 torque gearing)
constexpr double DR4B_TOWER_BACK_IN = 4.5; // main posts sit this far behind the front bumper
constexpr double DR4B_MID_GEAR_R = 0.75;   // the meshing gear pair between the two stages
constexpr double LIFT_MAX_DEG    = (DR4B_MAX_DEG - DR4B_MIN_DEG) * DR4B_GEAR;

// Lift type (settings): cascade 0.40 of the DR6B's reach and 1.6 times its speed, DR4B 0.65 of the reach at the DR6B's speed, DR6B the full reach
inline double g_liftScale = 1.0, g_liftSpeed = 1.0;
inline double g_wheelRpm = 450.0, g_topIps = 76.6;  // drivetrain wheel rpm (setting) and the top speed it gives on 3.25 in wheels, in/s
// Drive torque: every robot has a 4-motor drivetrain (two V5 blue 600 rpm motors a side) geared to the DRIVE RPM setting, so the wheel
// gets 600 / rpm times the motor torque. The force a side can push with falls toward zero as it nears top speed (and rises when it is
// being reversed), so a high-rpm drive accelerates slowly at speed and loses shoving contests, a low-rpm one punches. px/s^2, capped
// by tyre grip (0.8 g).
constexpr double DRIVE_MOTOR_TORQUE_NM = 0.3;  // usable torque per motor (2.1 N m stall, far less at the 2.5 A limit, with drivetrain losses and the voltage drop at speed): low enough that the rpm choice changes the launch, 200 rpm and 360 rpm hit the grip limit and 450 and 600 rpm do not
constexpr double ROBOT_MASS_KG = 5.3;
inline double driveAccelLimit(double wheelRpm, double v, double vmax, double dv) {
    const double r = 1.625 * 0.0254, G = 600.0 / wheelRpm;
    const double frac = std::clamp(std::fabs(v) / std::max(vmax, 1e-6), 0.0, 1.0);
    const double F = 2.0 * DRIVE_MOTOR_TORQUE_NM * G / r * ((dv * v >= 0.0) ? 1.0 - frac : 1.0 + frac);
    const double grip = 0.8 * 386.1 * (720.0 / 140.41);
    return std::clamp(F / (0.5 * ROBOT_MASS_KG) * 39.37 * (720.0 / 140.41), 0.05 * grip, grip);
}
inline int g_liftSections = 4, g_liftType = 2;  // sections drawn / used by the bars (a DR4B has two stages of one section, a DR6B two of two), and the type (0 cascade, 1 DR4B, 2 DR6B, 3 clawbot)
// Makes the lift globals those of this lift type; each robot sets its own before it is planned, moved or drawn
inline void setLiftType(int lt) {
    static const double LS[4] = {0.40, 0.50, 1.0, 0.45}, SP[4] = {1.6, 1.0, 1.0, 0.7};
    static const int SEC[4] = {2, 2, 3, 2};  // the DR6B drawn as the real lift: one long bar back (two bars' length), then two sections forward
    lt = std::clamp(lt, 0, 3);
    g_liftScale = LS[lt]; g_liftSpeed = SP[lt]; g_liftSections = SEC[lt]; g_liftType = lt;
}
// The computer plans its sticks as fractions of a 450 rpm drive (76.6 in/s). This turns one into the stick that gives the same real speed on
// a drive with a different top speed (a slow drive saturates, a fast one eases off until its command nears full, so it still cruises faster)
inline double driveNorm(double cmd, double topIps) {
    const double k = 76.6 / std::max(topIps, 1.0), a = std::fabs(cmd);
    if (k >= 1.0) return std::copysign(std::min(1.0, a * k), cmd);
    return std::copysign(a < 0.85 ? a * k : 0.85 * k + (a - 0.85) / 0.15 * (1.0 - 0.85 * k), cmd);
}
// Length of bar section k (pivot to pivot, in). A DR6B's first stage is one long bar twice a section (it runs on as the tower for the second
// stage's first section), so it goes back as far as the second stage's two sections come forward and the carriage rides straight up
inline double secLen(int k) { return (g_liftType == 2 && k == 0) ? 2.0 * DR4B_BAR_IN : DR4B_BAR_IN; }
inline double liftBarSum() { double s = 0.0; for (int k = 0; k < g_liftSections; ++k) s += secLen(k); return s; }
inline double dr4bAngleRad(double motorDeg) {
    double d = DR4B_MIN_DEG + std::clamp(motorDeg, 0.0, LIFT_MAX_DEG) / DR4B_GEAR;
    return d * M_PI / 180.0;
}
// Bottom of a held piece, inches above the floor
inline double dr4bLiftIn(double motorDeg) {
    return LIFT_BASE_IN + g_liftScale * DR4B_SECTIONS * DR4B_BAR_IN * (std::sin(dr4bAngleRad(motorDeg)) - std::sin(DR4B_MIN_DEG * M_PI / 180.0));
}
inline double dr4bMaxLiftIn() { return dr4bLiftIn(LIFT_MAX_DEG); }
// Height of the main posts' lower pivot, chosen so the claw (2 in above the piece bottom) sits on
// the carriage's lower pivot: each section rises bar x sin(angle), and each middle tower adds one spacing
inline double dr4bTowerPivotIn() {
    return LIFT_BASE_IN + 2.0 - (g_liftSections - 1) * DR4B_SPACING_IN
           - liftBarSum() * std::sin(DR4B_MIN_DEG * M_PI / 180.0);
}

// Toggle arm on the back of the DR6B: a rubber pad on an arm behind the middle tower, where the
// two stages meet. The middle tower swings back and up with the lift, so the pad's height and how
// far it sticks out change with the bar angle. Local inches: x forward of the robot's centre, z up.
inline void ramPadPose(double liftAngle, double halfLen, double& xLocal, double& zIn) {
    const double L = DR4B_BAR_IN, sp = DR4B_SPACING_IN;
    const double xt = halfLen - DR4B_TOWER_BACK_IN;
    if (g_liftType != 2) {  // cascade, DR4B and clawbot: the arms stand flush with the back bumper (inside the frame) on a riser; with the lift down the pads sit at 13.2 in, and they leave Toggle height as the lift rises
        const double deg = (liftAngle * 180.0 / M_PI - DR4B_MIN_DEG) * DR4B_GEAR;
        xLocal = -halfLen + 0.5;
        zIn = dr4bLiftIn(deg) + 2.0 + RAM_TALL_RISE_IN;
        return;
    }
    const double half = g_liftSections / 2;  // sections up to the middle tower
    double back = 0.0;
    for (int k = 0; k < half; ++k) back += secLen(k);
    xLocal = xt - back * std::cos(liftAngle) - RAM_ARM_BACK_IN;
    zIn = dr4bTowerPivotIn() + back * std::sin(liftAngle) + half * sp + RAM_ARM_RISE_IN;
}

// Ratcheting claw: the motor drives the jaws closed through a ratchet wheel. The pawl clicks
// over each tooth while closing and stops the jaws from opening, so the grip holds with the
// motor off (or the robot disabled). Opening first lifts the pawl, then the jaws swing open.
constexpr double CLAW_CLOSED_DEG = 200.0;  // claw motor travel from fully open to fully shut
constexpr int    RATCHET_TEETH   = 24;     // teeth over the full claw travel
constexpr double RATCHET_TOOTH_DEG = CLAW_CLOSED_DEG / RATCHET_TEETH;
constexpr double PAWL_LIFT_S     = 0.12;   // time to lift the pawl before the jaws can open
constexpr double JAW_OPEN_IN     = 5.0;    // gap between the jaws when fully open
constexpr double REACH_IN        = -4.0;   // held piece center, inches ahead of the front bumper (negative: inside the robot)
constexpr double JAW_LEN_IN      = REACH_IN + 1.5;

// Front opening: a slot in the front of the chassis, between the drive motors, that a goal (or a
// piece) drives into; the claw hangs over it. Robot-local inches from the front bumper.
constexpr double OPEN_HALF_IN  = 3.8;   // half its width
constexpr double OPEN_DEPTH_IN = 7.3;   // how far back it goes
// Goal alignment guide inside the opening: a shallow V of two angled arms at floor level, open
// toward the front. Driving onto a goal, its base slides along an arm, which shoves the robot
// sideways until the goal is centered; the back plate stops the goal with its center right at
// the claw's grip point. Robot-local inches: x ahead of the front bumper (negative = inside).
constexpr double ALIGN_GOAL_R    = 2.9;                       // goal base, between its flats and corners
constexpr double ALIGN_BACK_X    = REACH_IN - 2.805;          // back of the V: goal flat touches here
constexpr double ALIGN_BACK_HALF = 2.6;
constexpr double ALIGN_MOUTH_X   = 0.3, ALIGN_MOUTH_HALF = 3.7;
constexpr double ALIGN_Z0 = 0.2, ALIGN_Z1 = 0.9;              // height of the guide plates (under the jaws)

// ── Field and game ────────────────────────────────────────────────────────────
constexpr double PX_IN          = 720.0 / 140.41;
constexpr double PIECE_H_IN     = 6.5;
constexpr double HALF_IN        = 3.25;
// Piece shapes as (height, radius) profiles in inches, from VEX's official drawings
// (2026-27 Appendix A, "Pin Specifications" and "Cup Specifications", released 2026-04-27).
// The manual's text says "approximately 1.6 in" for a Pin, but the drawing shows a 3.16 in
// collar, 2.35 in body and 1.40 in tips, so the drawing is used.
struct ProfPt { double z, r; };
// Pin, 6.50 in: octagonal. Each half is a 1.40 in tip, straight for its first 0.64 in, then a
// taper out to 2.35 in where it meets the 3.16 in collar (lower edge 2.93 in from the tip).
// This is the bottom half, tip at z = 0; the top half is its mirror image. Each half of the
// collar is that half's color. The tips are hollow.
constexpr ProfPt PIN_HALF[] = {{0.00, 0.64}, {0.05, 0.70}, {0.64, 0.70}, {2.93, 1.175},
                               {2.93, 1.30}, {3.13, 1.58}, {3.25, 1.58}};
constexpr int    PIN_HALF_N = 7;
constexpr int    PIN_SIDES  = 8;
// Cup, 6.48 in (stacked on a 6.5 in pitch here): an open-ended, straight-sided hourglass, the same
// at both ends. A 3.16 in lipped rim at each end, tapering to a 2.32 in waist in the middle.
// Written with the opaque half at the bottom; a Pin half nests inside either half.
constexpr ProfPt CUP_PROF[] = {{0.00, 1.58}, {0.08, 1.58}, {0.36, 1.42}, {3.25, 1.16},
                               {6.14, 1.42}, {6.42, 1.58}, {6.50, 1.58}};
constexpr int    CUP_PROF_N = 7;
constexpr int    CUP_WAIST_I = 3;  // index of the waist, where the two halves meet
constexpr int    CUP_SIDES  = 16;
constexpr double CUP_WALL_IN = 0.07;

// Widths where the claw grips (its jaws are 2 in above the bottom of the piece)
constexpr double PIN_W_IN       = 1.96;    // pin body 2 in above its tip
constexpr double CUP_W_IN       = 2.55;    // cup body 2 in above its end
constexpr double PIECE_MAX_R_IN = 1.58;    // widest part of either piece (collar / rim), for pushing
constexpr double GOAL_W_IN      = 5.61;    // goal base across the flats (official drawing)
constexpr double MIDFIELD_PX    = 120.0;   // Midfield diamond, center to corner
constexpr double TOGGLE_HALF_PX = 13.0 * PX_IN;
constexpr int    SIDE_VIEW_H    = 120;     // side-view panel under the HUD, pixels

// Goal order matches Physics.cpp: 0-1 red, 2-3 blue, 4-7 neutral, 8 center
constexpr double GOAL_H_IN[9] = {3.25, 3.25, 3.25, 3.25, 5.77, 5.77, 5.77, 5.77, 8.77};
constexpr double GOAL_POS_IN[9][2] = {{23.11, 46.66}, {46.66, 23.11}, {117.30, 93.75}, {93.75, 117.30},
                                      {46.66, 117.30}, {23.11, 93.75}, {117.30, 46.66}, {93.75, 23.11},
                                      {70.20, 70.20}};

enum Col : uint8_t { C_RED, C_BLUE, C_YEL };

// ── Field markings and Loaders (official Appendix A drawings), field inches, origin bottom-left ──
constexpr double FIELD_IN     = 140.41;
constexpr double TILE_IN      = FIELD_IN / 6;
constexpr double LOADZONE_X   = TILE_IN / 2;   // load zone L: its upright runs down the middle of the corner tile
constexpr double TAPE_IN      = 1.0;           // tape width
constexpr double MIDFIELD_IN  = TILE_IN;       // Midfield diamond, center to corner
// Loaders: wall-mounted tubes at the corners of the red (left) and blue (right) walls. Dark base
// 4.02 in wide up to 11.85 in, clear 3.39 in tube above it to 22.97 in, reaching 3.74 in into the field.
struct LoaderDef { double x, y; Col c; };
constexpr LoaderDef LOADERS[4] = {{1.87, 11.44, C_RED}, {1.87, 128.97, C_RED},
                                  {FIELD_IN - 1.87, 11.44, C_BLUE}, {FIELD_IN - 1.87, 128.97, C_BLUE}};
constexpr double LOADER_W_IN = 4.02, LOADER_D_IN = 3.74, LOADER_BASE_H = 11.85, LOADER_H = 22.97, LOADER_TUBE_D = 3.39;
constexpr double LOADER_RISE_IN = 7.3;  // how far the clear tube rides up to open the bottom (a piece is 6.5 in tall)

// 3D camera views, cycled with C (next) and V (previous), or picked in the settings menu
constexpr int CAM_COUNT = 14;
inline const char* camName(int m) {
    static const char* names[CAM_COUNT] = {"CHASE", "RED STATION", "CLAW CAM", "OVERHEAD",
                                           "BLUE STATION", "AUDIENCE", "SIDE CHASE", "ORBIT",
                                           "FIRST PERSON", "LIFT CAM", "TOP FOLLOW", "BLEACHERS",
                                           "DRIVER VIEW", "LOADER CAM"};
    return names[((m % CAM_COUNT) + CAM_COUNT) % CAM_COUNT];
}
constexpr double STAGE_H_IN = 12.0;  // the field sits on a stage this high above the arena floor

// Settings menu (Tab): what each setting does is in menuChange below
struct SimSettings {
    bool outside = true;   // stage, bleachers, tables and the rest outside the field
    bool pose = false;     // robot position / heading readout on the 2D map
    bool dim = false;      // dim arena lighting
    bool trail = false;    // robot path trail on the 2D map
    bool drift = true;     // omni wheels slide sideways in hard turns
    int  turnPct = 100;    // how far a turn key pushes the right stick, percent (the robot code's TURN_SCALE applies on top)
    bool tipping = true;   // the robot can tip (and fall) from its centre of mass and accelerations
    bool physInfo = true;  // centre of mass, support area and tipping limits shown on the views
    int  bots = 5;         // other robots, an index into BOT_CHOICES (5 = all three, the default)
    std::array<bool, 4> omniBot{{false, false, false, false}};  // wheels per robot (partner, opponent 1, opponent 2, yours): false = traction middle wheels, true = all omni
    std::array<int, 3> level{{3, 2, 2}};  // experience of the red partner, blue opponent 1, blue opponent 2: 0 rookie .. 3 worlds
    int  pilot = 0;        // YOUR ROBOT: 0 = you drive it, 1-4 = the CPU drives it at ROOKIE .. WORLDS
    std::array<int, 4> focus{};  // FOCUS of the partner, opponent 1, opponent 2 and your robot (CPU): see BotFocus
    int  startSecs = 105;        // START TIME on the start screen: seconds on the match clock (10 to 600)
    bool infiniteLoads = false;  // MATCH LOADS on the start screen: false = 10 each (the rules), true = unlimited (for fun)
    int  hud = 3;          // over the 3D view: 0 off, 1 live scoreboard, 2 match timer, 3 both (the default)
    std::array<int, 4> liftBot{{2, 2, 2, 2}};  // lift type per robot (partner, opponent 1, opponent 2, yours): 0 cascade, 1 DR4B, 2 DR6B, 3 clawbot
    std::array<int, 4> rpmBot{{2, 2, 2, 2}};  // drivetrain wheel rpm per robot (partner, opponent 1, opponent 2, yours): 0 200, 1 360, 2 450, 3 600
    std::array<int, 4> yel{};  // per robot (partner, opponent 1, opponent 2, your robot's CPU): 0 = scores its own colour (alliance goals first, own colour showing), 1 = yellows (neutral and centre goals first, yellow showing)
    int  pilotLevel = 3;   // the CPU's level when it drives your robot (start screen), 0 rookie .. 3 worlds
};
constexpr int MENU_N = 22;
// What a CPU robot concentrates on (FOCUS settings)
enum BotFocus { FOCUS_SCORING, FOCUS_DEFENSE, FOCUS_TOGGLE_SCORE, FOCUS_N };
inline const char* focusName(int f) {
    static const char* n[FOCUS_N] = {"SCORING", "DEFENSE/TOGGLE", "TOGGLE/SCORING"};
    return n[std::clamp(f, 0, FOCUS_N - 1)];
}
// OTHER ROBOTS choices: which of the red partner (bit 0) and the blue opponents (bits 1, 2) are on
constexpr int BOT_CHOICE_N = 6;
constexpr int BOT_CHOICES[BOT_CHOICE_N] = {0, 1, 2, 6, 3, 7};
constexpr double MATCH_S = 105.0;  // driver control period, 1:45
inline double g_matchLen = MATCH_S;  // the length the START TIME setting gives (the match clock counts down from it)
// One click of START TIME: 5 s steps up to 1:00, 15 s steps up to 3:00, 30 s steps up to 10:00; it wraps round between 0:10 and 10:00
inline int startStep(int v, int dir) {
    if (dir > 0) { v += v < 60 ? 5 : (v < 180 ? 15 : 30); return v > 600 ? 10 : v; }
    v -= v <= 60 ? 5 : (v <= 180 ? 15 : 30);
    return v < 10 ? 600 : v;
}
constexpr int MATCH_LOAD_PINS = 10;  // match loads each alliance's drive team has for a match (pins, cups)
constexpr int MATCH_LOAD_CUPS = 10;

struct Piece {
    bool   isPin;
    double x, y;           // sim pixels (field 720 x 720, y down)
    Col    top, bot;       // pin halves (cups ignore these)
    bool   grayUp = true;  // cups: opaque half on top
    bool   held = false;
    int    goal = -1;      // goal index when stacked in a goal
    int    loader = -1;    // Loader index when dropped into a Loader
    int    placedBy = -1;  // robot (0-3) that last stacked it on a goal (points contributed)
    bool   lying = false;  // pins lying on their side: top = outer end, bot = inner end
    double ang = 0.0;      // lying pins: direction from inner to outer end (sim radians, y down)
    double vx = 0.0, vy = 0.0;  // sliding speed over the floor after a push, sim px/s
    double fz = 0.0, fzv = 0.0;  // height still to fall (in) and falling speed (in/s) after being let go
    double lz = 0.0, lzv = 0.0;  // in a Loader: current bottom height (in) and falling speed (down is +; - = bouncing up)
    double slide = 0.0;          // in a Loader: the bottom piece sliding out of the tube to the foot opening, 0 (in the tube) to 1 (out)
    // Tipping (standing pieces on the floor): lean about the edge of the base it rocks on
    double tilt = 0.0, tiltV = 0.0;  // lean (rad) and lean rate (rad/s)
    double tiltDir = 0.0;            // direction it leans toward (sim radians, y down)
    double tiltR = 0.0;              // base edge radius it pivots on (in): pin tip 0.70, cup rim 1.58
    int    inCup = -1;               // a standing pin in this standing cup's top: moves, leans and falls with it
    double spin = 0.0;               // lying: turning rate about the vertical (rad/s, sim sense), from off-centre hits
    double rollAng = 0.0;            // lying: how far it has rolled about its own axis (rad, for drawing)
    int    carrier = -1;             // carried by this other robot (Game::bots)
};

inline bool loose(const Piece& p) { return !p.held && p.goal < 0 && p.loader < 0 && p.carrier < 0; }

// Other robots (settings menu): copies of this robot (same drive, DR6B lift, roller claw, front
// opening and toggle arms) driven by the computer. Sim pixels and sim heading (0 = +x, y down), like
// the player's pose.
constexpr double PLAYER_TORQUE_YOU = 1.15;  // the player's drive has 15 percent more torque than the other robots while a person drives it
#define PLAYER_TORQUE (game().set.pilot > 0 ? 1.0 : PLAYER_TORQUE_YOU)  // (with the CPU driving it, all robots are equal)
constexpr int    BOT_N        = 4;  // 0 red partner, 1-2 blue, 3 a CPU copy of the player's robot (simulated tournament matches)
constexpr int    BOT_N_OLD    = 3;                       // 0 = red partner, 1 and 2 = blue opponents
constexpr double BOT_HALF_IN  = 9.0;                     // 18 x 18 in, like the player's robot
constexpr double BOT_GRIP_IN  = BOT_HALF_IN + REACH_IN;  // claw grip point, ahead of the robot's centre
constexpr double BOT_TURN_SCALE = 0.4;                   // the robot code's TURN_SCALE (split arcade)
constexpr double BOT_LIFT_DPS = 0.70 * 200.0 * 6.0;      // lift motor deg/s: the robot code's LIFT_MAX_SPEED 70 on green 200 rpm
struct Bot {
    bool   on = false;
    Col    team = C_RED;
    const char* num = "";
    double x = 0, y = 0, h = 0;     // sim px, sim radians
    double vL = 0, vR = 0;          // left and right wheel surface speeds, px/s
    double vx = 0, vy = 0;          // its velocity over the field, px/s (the sideways part slides off at the wheels' grip)
    double liftDeg = 0, liftTarget = 0;  // lift motor degrees, like the player's lift motor
    double clawDeg = 0;             // claw roller turn (drawing)
    double wheelRotL = 0, wheelRotR = 0;  // wheel turn (drawing)
    int    carry = -1;              // piece in its claw
    int    apTarget = -1;           // piece its approach line was worked out for
    bool   apFinal = false;         // lined up behind the piece: driving straight in
    int    rider = -1;              // a second piece nested on it (a pin and a cup taken together from a Loader)
    int    target = -1;             // piece it is going for
    int    goal = -1;               // goal it is taking its piece to
    int    skip = -1;               // a piece it gave up on, left alone for a while
    double skipT = 0, wait = 0;     // seconds left on the skip, and on a pause (the CPU driver hesitating)
    double stuckT = 0, chkX = 0, chkY = 0;  // stuck check: time since the last check and where it was
    double backT = 0, backTurn = 0;  // backing out after placing or getting stuck
    double defendT = 0, restT = 0;  // hard: time spent defending, and before it may defend again
    int    posture = 0;             // how the match is going for its alliance: 0 even, 1 losing by a lot (go risky: yellows and Toggles), 2 well ahead (protect the lead: defend)
    double postureT = 0, pinT = 0, pinCool = 0;  // time to the next posture check; time it has been pinning an opponent; time before it can be called again
    bool   pinNow = false;          // pinning an opponent this step
    double dropT = 0;                         // the CPU-driven player robot: run the claw out for this long to let a piece go
    bool   scoreYel = false;                  // SCORES setting: yellows (else its own colour)
    bool   omni = false;                      // all omni wheels (else traction middle wheels)
    bool   dumping = false;                   // carrying a piece it can't use to a spot off its paths to drop it there
    double dumpX = 0, dumpY = 0, dumpT = 0;   // that spot (px) and how long it has been going there
    double topIps = 76.6;                     // its drivetrain top speed, in/s (DRIVE RPM setting)
    int    lift = 2;                          // its lift type (LIFT setting)
    double protT = 0;                          // seconds in contact with a robot protected in its Load Zone (a violation after 3 s)
    double dsX = 0, dsY = 0, dsT = 0;         // holding a piece without moving: where it was and for how long
    double carryT = 0, noGoalT = 0;           // time it has held its piece, and with no goal that takes it
    double escapeT = 0, escX = 0, escY = 0;  // penalty avoidance: driving away from an opposing goal or Load Zone toward (escX, escY)
    double targetT = 0;             // time spent going for the current piece (it gives up after a while)
    double zoneT = 0;               // time it has stayed in the other alliance's Load Zone (<SG13>)
    double goalT = 0;               // time it has been touching an opposing Alliance Goal (<SG9>)
    double hitCool = 0;             // after contacting a protected robot: no second call for a moment (<SG13a>)
    std::array<bool, 4> touchTog{}; // its toggle arms are touching this Toggle
    // Expert: what it is doing (0 = a loose piece or carrying, 1 = taking from a Loader, 2 = turning a
    // Toggle), which Loader or Toggle, and the step it is on
    int    task = 0, taskK = -1, phase = 0;
    double taskT = 0, phaseT = 0, replanT = 0;
    bool   tgHit = false;               // its toggle arms just rolled a Toggle
    double wdT = 0, wdX = 0, wdY = 0;   // watchdog: time it has stayed within 4 in of (wdX, wdY)
    double wanderT = 0, wanderX = 0, wanderY = 0;  // after being stuck: drive to an open spot for a moment
    bool   endgame = false;             // it has switched to its endgame plan (the Toggles)
    int    focus = 0;                   // BotFocus in use
    int    cyc = 0, cycLi = -1, cycGoal = -1;  // SCORING focus: its phase, its Loader and goal (CyclePhase)
    double cycT = 0, cycOff = 0, togCool = 0, dockWait = 0;  // time in phase, time the cycle is off, since the last Toggle trip, waiting at the dock
    double cycX0 = 0, cycY0 = 0, cycX1 = 0, cycY1 = 0;      // the straight leg it is driving (sim px)
    double cycRx = 0, cycRy = 0;                             // the pivot: where it turned toward the goal
    double cycPH = 0, cycW = 0;                              // last heading and smoothed yaw rate
    int    level = 2, focusSet = 0;     // its experience (0 rookie .. 3 worlds) and FOCUS, set when it is placed
    int    placed = 0, loaded = 0;      // pieces it has stacked, and taken from a Loader (test readout)
    std::array<bool, 4> ramArmed{{true, true, true, true}};  // a ram counts once per Toggle; back off to re-arm
};
// OPP EXPERIENCE (the DIFFICULTY row) is how experienced the other teams are, in how they drive and
// how they play:
//   ROOKIE   - slow, hesitant drivers who put the nearest piece on their own goal
//   REGIONAL - better drivers who weigh what a piece is worth, use their Loaders, and go for the
//              Toggles in the last 20 s
//   STATE    - quick, decisive drivers with the same plan, Toggles from 35 s out, and defense
//   WORLDS   - the best drivers: no wasted time, Toggles from 40 s out, defense
// Each robot has its own level (settings PARTNER LEVEL, OPP 1 LEVEL, OPP 2 LEVEL, YOUR ROBOT).
// stick: how hard it pushes the sticks; pause: hesitation after a pickup or placement; smart: plans by
// points per second over any goal it may use; loaders: takes pieces from its Loaders (and its drive
// team match loads them); endgame: seconds left when it turns to the Toggles (0 = never); approach:
// lowest stick on the last stretch; lineUp: how carefully it lines up first; liftEarly: how far out
// it starts raising the lift
struct BotSkill { double stick, pause; bool defends, smart, loaders; double endgame, approach, lineUp, liftEarly;
                  double grabOK, placeOK, idle, dock, backOff, reload; };
// grabOK / placeOK: chance a pickup / a placement works (a miss: it backs off; a dropped piece falls to the
// floor); idle: extra seconds it dawdles after each placement; dock: stick while docking at a Loader;
// backOff: seconds it backs away after placing; reload: seconds its drive team takes between match loads
inline BotSkill botSkill(int) {
    // Every CPU robot plays at one level: WORLDS. Full stick, no hesitation, never misses a pickup or a
    // placement, lift rises early, the drive team reloads every 0.6 s.
    static const BotSkill s = {1.00, 0.00, true, true, true, 40.0, 0.60, 3.0, 80.0, 1.00, 1.00, 0.0, 0.50, 0.30, 0.6};
    return s;
}
inline BotSkill botSkillFor(const std::array<int, 3>& level, int bi) { return botSkill(level[std::clamp(bi, 0, 2)]); }
inline double widthIn(const Piece& p) { return p.isPin ? PIN_W_IN : CUP_W_IN; }
inline double floorRadiusIn(const Piece& p) { return p.lying ? 2.0 : PIECE_MAX_R_IN; }
inline double jawGapIn(double clawDeg) { return JAW_OPEN_IN * (1.0 - std::clamp(clawDeg, 0.0, CLAW_CLOSED_DEG) / CLAW_CLOSED_DEG); }
inline double gripDeg(double pieceW) { return CLAW_CLOSED_DEG * (1.0 - pieceW / JAW_OPEN_IN); }

// Tournament (start screen: TOURNAMENT): 16 alliances of two teams, single elimination
struct BTeam { std::string num; int level = 3, focus = 0, rpm = 2, lift = 2; bool omni = false, yel = false; };  // yel: scores yellow (else its own colour)  // rpm: 0 200, 1 360, 2 450, 3 600; lift: 0 cascade .. 3 clawbot
// The 60 teams that can play (the all-time board lists all of them); each tournament draws 40. Teams share organisations (same number, other letter): every organisation has 2 or 3 teams
static const char* const TEAM_POOL[60] = {
    "41658A", "41658C", "41658F", "1010A", "1010B", "1010C", "5225A", "5225E", "5225X", "9364A",
    "9364B", "9364C", "7701A", "7701D", "7701F", "8812A", "8812B", "8812X", "2024A", "2024F",
    "2024X", "3141A", "3141B", "3141C", "4420B", "4420C", "4420F", "2210A", "2210B", "2210C",
    "1155A", "1155D", "1155E", "1234B", "1234C", "1234D", "2775B", "2775X", "4097C", "4097D",
    "3003A", "3003B", "6150B", "6150F", "9090A", "9090D", "5858B", "5858D", "1701A", "1701X",
    "6677A", "6677F", "8080B", "8080C", "3399C", "3399F", "7272A", "7272D", "4545B", "4545X"};
struct BAlliance { BTeam t[2]; };
struct BMatch { int a = -1, b = -1;       // alliances (a = red, the better seed)
                int ra = -1, rb = -1;     // final scores
                int winner = -1; };
// Tournament match records (stats): one per finished match, kept for the bracket and appended to vexsim_stats.jsonl
struct TeamMatchStat { std::string num; int alliance = 0; bool red = true; int placed = 0, loaded = 0, rpm = 2, lift = 2, focus = 0, toggles = 0, points = 0; bool omni = false, yel = false; };
struct MatchRec { int match = 0, redAl = 0, blueAl = 0, redScore = 0, blueScore = 0, winnerAl = 0; std::array<TeamMatchStat, 4> t; };
constexpr int BR_ALLIANCES = 8, BR_MATCHES = 7;  // quarterfinals (0-3), semifinals (4-5), final (6)
// 40 teams play qualification matches; the top 16 form the 8 alliances above. A qualification match runs as match QM
// (a scratch match) between two scratch alliances (BR_ALLIANCES and BR_ALLIANCES + 1) built from the schedule.
constexpr int QM = 7, QUAL_N = 40, QUAL_ROUNDS = 5;
struct QStat { int w = 0, l = 0, t = 0, pts = 0; int wp() const { return 2 * w + t; } int played() const { return w + l + t; } };

struct Game {
    std::vector<Piece> p;
    std::array<std::vector<int>, 9> stack;
    int    held = -1;         // piece gripped by the claw
    int    rider = -1;        // piece nested on the gripped one (one pin and one cup at most)
    bool   ratchetArmed = true;  // ratchet latch ready to snap onto the next piece pushed into the jaws
    bool   inited = false;
    int    overGoal = -1;
    bool   overGoalOk = false;
    double liftIn = LIFT_BASE_IN;
    double liftAngle = DR4B_MIN_DEG * M_PI / 180.0;  // DR4B bar angle, radians
    double gapIn = 0.0;
    double prevGapIn = 0.0;
    double clawDeg = 0.0;      // claw motor position (turns the wheel stacks, unlimited)
    double clawRpm = 0.0;      // wheel stack speed: + pulls a piece in, - pushes it out
    double pawlUp = 0.0;       // 0 = pawl engaged, 1 = fully lifted (opening)
    double pawlClick = 0.0;    // 0..1 bounce of the pawl as it rides over a tooth while closing

    // Toggles: 0 left wall, 1 right, 2 bottom, 3 top (one per quadrant)
    std::array<Col, 4>  toggle{{C_YEL, C_YEL, C_YEL, C_YEL}};
    std::array<bool, 4> touching{};      // the spinner is against this Toggle
    std::array<double, 4> toggleTurn{};  // 0..1 progress of the roll to the next face
    std::array<bool, 4> ramArmed{{true, true, true, true}};  // a ram counts once; back off to re-arm
    std::array<double, 4> ramPrevAcross{{99, 99, 99, 99}};  // ram tip to Toggle distance last step
    double wheelRotL = 0.0, wheelRotR = 0.0;  // drive wheel spin for drawing, radians
    double prevRx = 0.0, prevRy = 0.0;          // robot position last step (for its velocity)
    double prevRh = 0.0;                        // robot heading last step (for its turn rate)
    double heldOffX = 0.0, heldOffY = 0.0;      // a just-grabbed piece's offset from the claw's grip point, sim px; eases to 0
    bool   prevRValid = false;

    // Loaders: pieces dropped in from the top, bottom one first (it comes out at the floor)
    std::array<std::vector<int>, 4> loaderStack;
    int loadRequest = 0;  // match load from the drive team: 1 = a pin, 2 = a cup (Z / X keys)
    std::array<double, 4> loaderGate{};  // clear cover over each Loader's foot opening: 0 = down (closed), 1 = lifted

    int  red = 0, blue = 0;
    int  midRed = 0, midBlue = 0;   // robots of each alliance at least partly in the Midfield right now

    bool     resetRequested = false;
    double   sx = 0, sy = 0, sh = 0;
    int      camMode = 5;  // 3D view, see camName(); starts on the AUDIENCE view
    SimSettings set;
    bool     menuOpen = false;
    int      menuSel = 0;
    std::vector<std::pair<float, float>> trail;  // robot centre every 0.1 s, sim px
    double   trailT = 0.0;

    // Robot mass properties (robotMassProps), robot-frame inches: x forward of the centre, y left, z up
    double massLb = 13.0, comX = 0.0, comY = 0.0, comZ = 5.0, k2Pitch = 40.0, k2Roll = 30.0;
    // Tipping: pitch > 0 = nose down about the front wheels, < 0 = tail down about the back wheels;
    // roll > 0 = over to the left wheels, < 0 = to the right. Radians and rad/s.
    double pitch = 0.0, pitchV = 0.0, roll = 0.0, rollV = 0.0;
    bool   tipped = false;       // fell over: the wheels are off the floor until a reset (R)
    double tipBrakeG = 1.0, tipAccelG = 1.0, tipSideG = 1.0;  // steady acceleration (in g) that would tip it
    double accF = 0.0, accL = 0.0;  // robot-frame acceleration last step, in/s^2 (for the readout)
    std::array<Bot, BOT_N> bots{};  // other robots (settings: OTHER ROBOTS and their levels)
    Bot pilot;                      // the CPU driving the player's robot (YOUR ROBOT setting)
    std::array<Col, 4> pilotPrevToggle{};
    bool botsDirty = true;          // place them again (after a setting change or a reset)
    std::array<bool, 4> botAtLoader{};  // another robot is docked at this Loader (its cover lifts)
    std::array<double, 4> loadT{};      // expert: time since the CPU drive team's last match load into this Loader
    bool   timeEdit = false;            // START TIME is being typed on the start screen
    std::string timeBuf;                // what has been typed so far (digits and one ':')
    bool   endRequested = false;        // F8: end the match now
    double skipTo = 0.0;                // F9: jump the clock to this many seconds left (0 = no request)
    double matchT = MATCH_S;            // match clock: seconds left in driver control (the robots plan by it)
    std::array<int, 2> loadPins{{MATCH_LOAD_PINS, MATCH_LOAD_PINS}}, loadCups{{MATCH_LOAD_CUPS, MATCH_LOAD_CUPS}};  // match loads left: red, blue
    double playerProtT = 0.0;           // the player's robot touching a protected robot
    bool   matchOver = false;
    bool   nextRequested = false;       // NEXT pressed (Enter, Space or a click) after a match: only then does the start screen come back
    bool   dash = true;                 // the start screen (MATCH SETUP) is showing
    int    infoAl = 0, infoFrom = 1;    // alliance whose hardware page is open (dashPage 4), and the page it came from
    int    dashPage = 0;                // 0 = match setup, 1 = tournament bracket
    std::array<BAlliance, BR_ALLIANCES + 2> bAl{};
    std::array<BMatch, BR_MATCHES + 1> bM{};
    std::array<int, 4> togOwner{{-1, -1, -1, -1}};  // robot that last rolled each Toggle this match
    std::vector<std::array<int, 3>> qRes;  // qualification results: red score, blue score, winner side (0 red, 1 blue, 2 tie); -1 not played
    int    qCount = 5, simLeft = -1;    // SIM NEXT N: how many (typed on the qualification page) and how many are still to play (-1: no limit)
    bool   qCountEdit = false;
    std::string qCountBuf;
    int    qView = 0, qMPage = -1, qSPage = 0;  // qualification page: the match results page (-1 follows the latest match) and the standings page (0: teams 1-20, 1: 21-40)
    int    lbPage = 0;                  // leaderboards: page of 40 teams
    int    lbMode = 0;                  // leaderboards: 0 this tournament, 1 all-time
    int    stage = 0;                   // tournament: 0 picking the 40 teams' robots, 1 qualification, 2 elimination
    std::vector<BTeam> teams;           // the 40 teams
    std::vector<std::array<int, 4>> qSched;  // qualification schedule: red team, red team, blue team, blue team (indices into teams)
    int    qDone = 0;                   // qualification matches played
    std::array<QStat, QUAL_N> qs{};     // qualification results per team
    int    teamPage = 0;                // page of the 40-team list
    bool   autoAll = false;             // after the last qualification match, form alliances and simulate the elimination too
    bool   bracketMade = false;
    std::vector<MatchRec> recs;         // finished tournament matches of this bracket (stats)
    std::vector<MatchRec> allRecs;      // every match in vexsim_stats.jsonl (the all-time leaderboard), read when the leaderboard opens
    long   bracketId = 0;               // this bracket's id in the stats file
    int    bracketCur = -1;             // tournament match being played (-1: none)
    bool   fastSim = false;             // simulating that match without showing it (many steps per frame)
    bool   simAll = false;              // keep simulating until the bracket is finished
    bool   playerOff = false;           // the player's robot sits this match out (a CPU copy, bot 3, plays instead)
    bool   playRequested = false;       // PLAY was pressed on it
    double overT = 0.0;                 // time since the match ended (the start screen comes back after a few seconds)
    int    lastRed = -1, lastBlue = -1;  // the last match's score, shown on the start screen
    bool   lastSimulated = false;       // it was a SIM MATCH
    bool   previewFast = true;          // the match preview came from SIM NEXT (else WATCH NEXT)
    std::array<int, 2> minors{};        // Minor Violations this match: red, blue
    std::array<bool, 2> dq{};           // Disqualified this match (a Major Violation): red, blue
    double playerZoneT = 0.0;           // time the player's robot has stayed in a blue Load Zone
    double playerGoalT = 0.0, playerLiftT = 0.0, playerHitCool = 0.0;  // <SG9>, <SG12>, <SG13a> timers for the player's robot
    std::array<int, 2> loadYellow{{1, 1}};  // yellow/yellow match loads left: red, blue
    std::string violMsg;                // the referee's last call, shown for a few seconds
    double violMsgT = 0.0;
    int    dashSel = -1;                // start screen: widget picked with the arrow keys (-1 = PLAY)
};

struct GameResult {
    bool robotMoved = false;  // reset or blocked by a goal rim: Physics drops its wheel speeds
    int  pushed = 0;          // loose pieces the robot is shoving this step
};

inline Game& game() { static Game g; return g; }

inline const char* menuLabel(int i) {
    static const char* labels[MENU_N] = {"OUTSIDE FIELD", "POSITION", "LIGHTING", "PATH TRAIL",
                                         "DRIFTING", "TURN SPEED", "CAMERA", "TIPPING", "PHYSICS INFO",
                                         "OTHER ROBOTS", "YOUR ROBOT", "PARTNER FOCUS", "OPP 1 FOCUS", "OPP 2 FOCUS", "YOUR FOCUS",
                                         "DRIVE RPM", "SCOREBOARD", "PARTNER SCORES", "OPPONENTS SCORE", "CLEAR TRAIL", "RESET FIELD", "LIFT TYPE"};
    return labels[i];
}
inline const char* menuValue(int i) {
    static char buf[32];
    const SimSettings& st = game().set;
    switch (i) {
        case 0: return st.outside ? "ON" : "OFF";
        case 1: return st.pose ? "ON" : "OFF";
        case 2: return st.dim ? "DIM" : "BRIGHT";
        case 3: return st.trail ? "ON" : "OFF";
        case 4: return st.drift ? "ON" : "OFF";
        case 5: std::snprintf(buf, sizeof(buf), "%d%%", st.turnPct); return buf;
        case 6: return camName(game().camMode);
        case 7: return st.tipping ? "ON" : "OFF";
        case 8: return st.physInfo ? "ON" : "OFF";
        case 9: { static const char* b[BOT_CHOICE_N] = {"NONE", "PARTNER", "1 OPPONENT", "2 OPPONENTS", "PARTNER + 1 OPP", "ALL 3"};
                  return b[std::clamp(st.bots, 0, BOT_CHOICE_N - 1)]; }
        case 10: return st.pilot > 0 ? "CPU DRIVES" : "YOU DRIVE";
        case 11: case 12: case 13: case 14: return focusName(st.focus[i - 11 == 3 ? 3 : i - 11]);
        case 15: { for (int k = 1; k < 4; ++k) if (st.rpmBot[k] != st.rpmBot[0]) return "MIXED"; return st.rpmBot[0] == 1 ? "360" : "450"; }
        case 16: { static const char* h[4] = {"OFF", "SCOREBOARD", "TIMER", "BOTH"}; return h[st.hud & 3]; }
        case 17: return st.yel[0] == 0 ? "RED" : "YELLOW";
        case 18: return st.yel[1] == 0 ? "BLUE" : "YELLOW";
        case 21: { static const char* lt[4] = {"CASCADE", "DR4B", "DR6B", "CLAWBOT"}; for (int k = 1; k < 4; ++k) if (st.liftBot[k] != st.liftBot[0]) return "MIXED"; return lt[std::clamp(st.liftBot[0], 0, 3)]; }
        default: return "PRESS ENTER";
    }
}
// Change setting i one step (dir = +1 / -1); the two actions run on either
inline void menuChange(int i, int dir) {
    Game& g = game();
    SimSettings& st = g.set;
    switch (i) {
        case 0: st.outside = !st.outside; break;
        case 1: st.pose = !st.pose; break;
        case 2: st.dim = !st.dim; break;
        case 3: st.trail = !st.trail; break;
        case 4: st.drift = !st.drift; break;
        case 5: st.turnPct = std::clamp(st.turnPct + 20 * dir, 40, 100); break;
        case 6: g.camMode = (g.camMode + dir + CAM_COUNT) % CAM_COUNT; break;
        case 7: st.tipping = !st.tipping; g.pitch = g.pitchV = g.roll = g.rollV = 0.0; g.tipped = false; break;
        case 8: st.physInfo = !st.physInfo; break;
        case 9: st.bots = (st.bots + dir + BOT_CHOICE_N) % BOT_CHOICE_N; g.botsDirty = true; break;
        case 10: st.pilot = st.pilot > 0 ? 0 : 1; break;
        case 11: case 12: case 13: case 14: st.focus[i - 11] = (st.focus[i - 11] + dir + FOCUS_N) % FOCUS_N; break;
        case 15: { const int v = st.rpmBot[0] == 1 ? 2 : 1; st.rpmBot = {{v, v, v, v}}; break; }
        case 16: st.hud = (st.hud + dir + 4) % 4; g.matchT = g_matchLen; g.matchOver = false; break;
        case 17: st.yel[0] = st.yel[3] = 1 - st.yel[0]; break;
        case 18: st.yel[1] = st.yel[2] = 1 - st.yel[1]; break;
        case 19: g.trail.clear(); break;
        case 20: g.resetRequested = true; g.menuOpen = false; break;
        case 21: { const int v = (st.liftBot[0] + dir + 3) % 3; st.liftBot = {{v, v, v, v}}; break; }
    }
}

// ── Tournament bracket ──
// 8 alliances of two teams, single elimination. Seed 1 is the player's alliance (its robot and partner,
// focuses from the start screen); the others get made-up team numbers and the BALANCED focus. Every team
// plays at the same level (WORLDS). Quarterfinals: 1v8, 4v5, 3v6, 2v7; the better seed plays red.
// Teams 1 and 2 are the player's (41658F and 41658A); the other 38 are drawn at random from the pool of 60
inline void pickTeams(Game& g) {
    std::vector<int> idx;
    for (int k = 0; k < 60; ++k)
        if (std::string(TEAM_POOL[k]) != "41658F" && std::string(TEAM_POOL[k]) != "41658A") idx.push_back(k);
    for (int k = (int)idx.size() - 1; k > 0; --k) std::swap(idx[k], idx[std::rand() % (k + 1)]);
    for (int i = 2; i < QUAL_N; ++i) g.teams[i].num = TEAM_POOL[idx[i - 2]];
}
// RANDOMIZE: a new random 40 teams and new random robots (the player's two keep their robots)
inline void randomizeTeams(Game& g) {
    pickTeams(g);
    for (int i = 2; i < QUAL_N; ++i) { BTeam& t = g.teams[i]; t.rpm = 1 + std::rand() % 2; t.lift = std::rand() % 3; t.omni = false; }
}
inline void bracketNew(Game& g) {
    g.teams.assign(QUAL_N, BTeam{});
    for (int i = 0; i < QUAL_N; ++i) {
        BTeam& t = g.teams[i];
        t.level = 3;
        t.focus = FOCUS_SCORING;  // a team's focus is picked on the match preview, before each elimination match
        t.rpm = 2; t.lift = 2; t.omni = false; t.yel = false;  // drivetrain rpm, lift and wheels are picked on the teams page
        if (i < 2) {
            t.num = i == 0 ? "41658F" : "41658A";
            t.focus = i == 0 ? g.set.focus[3] : g.set.focus[0];
            const int ri = i == 0 ? 3 : 0;
            t.rpm = g.set.rpmBot[ri]; t.lift = g.set.liftBot[ri]; t.omni = g.set.omniBot[ri]; t.yel = g.set.yel[ri] != 0;
            continue;
        }
        t.num = "";
    }
    pickTeams(g);
    for (auto& A : g.bAl) A = BAlliance{};
    for (auto& m : g.bM) m = BMatch{};
    g.simLeft = -1; g.stage = 0; g.qDone = 0; g.qSched.clear(); g.qs = {}; g.teamPage = 0; g.autoAll = false;
    if (const char* f = std::getenv("VEXSIM_FOCUS"))  // testing: every team this focus (0 balanced, 1 scoring, 2 toggle/defense)
        for (auto& t : g.teams) t.focus = std::clamp(std::atoi(f), 0, FOCUS_N - 1);
    if (const char* f = std::getenv("VEXSIM_FOCUS_EVEN"))  // testing: the even-numbered alliances' first team
        for (int i = 2; i < QUAL_N; i += 2) g.teams[i].focus = std::clamp(std::atoi(f), 0, FOCUS_N - 1);
    if (std::getenv("VEXSIM_INFINITE")) g.set.infiniteLoads = true;  // testing: unlimited match loads
    g.bracketMade = true;
    g.simAll = false;
    g.recs.clear();
    g.bracketId = (long)std::time(nullptr);
}
// Next match to play (in order), or -1 when the bracket is finished
inline int bracketNext(const Game& g) {
    for (int k = 0; k < BR_MATCHES; ++k)
        if (g.bM[k].a >= 0 && g.bM[k].b >= 0 && g.bM[k].winner < 0) return k;
    return -1;
}
inline int qualTotal(const Game& g) { return (int)g.qSched.size(); }
// The schedule: QUAL_ROUNDS rounds, each shuffling the 40 teams into 10 matches of two against two
inline void qualBuild(Game& g) {
    g.qSched.clear(); g.qDone = 0; g.qs = {};
    for (int r = 0; r < QUAL_ROUNDS; ++r) {
        int o[QUAL_N];
        for (int k = 0; k < QUAL_N; ++k) o[k] = k;
        for (int k = QUAL_N - 1; k > 0; --k) std::swap(o[k], o[std::rand() % (k + 1)]);
        for (int k = 0; k < QUAL_N; k += 4) g.qSched.push_back({o[k], o[k + 1], o[k + 2], o[k + 3]});
    }
    g.qRes.assign(g.qSched.size(), std::array<int, 3>{{-1, -1, -1}});
    g.qView = 0; g.qMPage = -1; g.qSPage = 0;
    g.stage = 1;
}
// Team indices, best first: win points (2 a win, 1 a tie), then total score
inline std::vector<int> qualRanking(const Game& g) {
    std::vector<int> v(QUAL_N);
    for (int k = 0; k < QUAL_N; ++k) v[k] = k;
    std::stable_sort(v.begin(), v.end(), [&](int a, int b) {
        if (g.qs[a].wp() != g.qs[b].wp()) return g.qs[a].wp() > g.qs[b].wp();
        return g.qs[a].pts > g.qs[b].pts;
    });
    return v;
}
inline void qualStart(Game& g, bool fast) {
    if (g.qDone >= qualTotal(g)) return;
    const std::array<int, 4>& sc = g.qSched[g.qDone];
    g.bAl[BR_ALLIANCES].t[0] = g.teams[sc[0]]; g.bAl[BR_ALLIANCES].t[1] = g.teams[sc[1]];
    g.bAl[BR_ALLIANCES + 1].t[0] = g.teams[sc[2]]; g.bAl[BR_ALLIANCES + 1].t[1] = g.teams[sc[3]];
    BMatch& m = g.bM[QM];
    m = BMatch{};
    m.a = BR_ALLIANCES; m.b = BR_ALLIANCES + 1;
    g.bracketCur = QM;
    g.fastSim = fast;
    g.playerOff = true;
    g.botsDirty = true;
    g.playRequested = true;
    g.dash = false;
}
// Alliance selection: the top 8 teams are captains, and each picks the best team left (ranks 9 to 16 in turn)
inline void formAlliances(Game& g) {
    const std::vector<int> r = qualRanking(g);
    for (int i = 0; i < BR_ALLIANCES; ++i) { g.bAl[i].t[0] = g.teams[r[i]]; g.bAl[i].t[1] = g.teams[r[BR_ALLIANCES + i]]; }
    static const int QF[4][2] = {{0, 7}, {3, 4}, {2, 5}, {1, 6}};
    for (int k = 0; k < BR_MATCHES; ++k) g.bM[k] = BMatch{};
    for (int k = 0; k < 4; ++k) { g.bM[k].a = QF[k][0]; g.bM[k].b = QF[k][1]; }
    g.stage = 2;
    g.simAll = false;
}
inline bool bracketDone(const Game& g) { return g.stage == 2 && g.bM[6].winner >= 0; }
// Start the next match, simulated (fast) or watched. Every tournament match is four CPU robots (a CPU
// copy plays for the player's robot).
inline void bracketStart(Game& g, bool fast) {
    const int k = bracketNext(g);
    if (k < 0) return;
    g.bracketCur = k;
    g.fastSim = fast;
    g.playerOff = true;
    g.botsDirty = true;
    g.playRequested = true;
    g.dash = false;
}
inline void scoreField(Game& g, int& red, int& blue, std::array<int, 4>* by = nullptr);
// The finished match's score and winner (a tie goes to a coin toss), and the winner moves on
inline void bracketRecord(Game& g) {
    BMatch& m = g.bM[g.bracketCur];
    if (m.winner >= 0) return;
    m.ra = g.red; m.rb = g.blue;
    if (std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "RESULT match %d red #%d %d blue #%d %d\n", g.bracketCur, m.a + 1, g.red, m.b + 1, g.blue);
    if (std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "DETAIL placed r0 %d r3 %d b1 %d b2 %d loaded r0 %d r3 %d b1 %d b2 %d toggles %d %d %d %d mid %d %d stacks %zu %zu %zu %zu | %zu %zu %zu %zu %zu\n", g.bots[0].placed, g.bots[3].placed, g.bots[1].placed, g.bots[2].placed, g.bots[0].loaded, g.bots[3].loaded, g.bots[1].loaded, g.bots[2].loaded, (int)g.toggle[0], (int)g.toggle[1], (int)g.toggle[2], (int)g.toggle[3], g.midRed, g.midBlue, g.stack[0].size(), g.stack[1].size(), g.stack[2].size(), g.stack[3].size(), g.stack[4].size(), g.stack[5].size(), g.stack[6].size(), g.stack[7].size(), g.stack[8].size());
    m.winner = g.red > g.blue ? m.a : g.blue > g.red ? m.b : ((std::rand() & 1) ? m.a : m.b);
    if (g.dq[0] != g.dq[1]) m.winner = g.dq[0] ? m.b : m.a;  // a Disqualified alliance loses
    const bool qual = g.bracketCur == QM;
    const bool tie = qual && g.red == g.blue && g.dq[0] == g.dq[1];
    {   // stats: this match's record, kept in memory for the alliance pages and appended to vexsim_stats.jsonl
        MatchRec r;
        r.match = g.bracketCur; r.redAl = m.a; r.blueAl = m.b; r.redScore = g.red; r.blueScore = g.blue; r.winnerAl = tie ? -1 : m.winner;
        const BAlliance& A = g.bAl[m.a];
        const BAlliance& B = g.bAl[m.b];
        const BTeam* tt[4] = {&A.t[1], &B.t[0], &B.t[1], &A.t[0]};  // bots 0..3
        std::array<int, 4> by{};
        { int rr, bb; scoreField(g, rr, bb, &by); }
        for (int i = 0; i < 4; ++i) {
            TeamMatchStat& x = r.t[i];
            x.num = tt[i]->num; x.alliance = (i == 1 || i == 2) ? m.b : m.a; x.red = !(i == 1 || i == 2);
            x.placed = g.bots[i].placed; x.loaded = g.bots[i].loaded;
            for (int k = 0; k < 4; ++k) if (g.togOwner[k] == i && g.toggle[k] == g.bots[i].team) ++x.toggles;  // Toggles it rolled that are still its colour at the end
            x.points = by[i];
            x.rpm = tt[i]->rpm; x.lift = tt[i]->lift; x.focus = tt[i]->focus; x.omni = tt[i]->omni; x.yel = tt[i]->yel;
        }
        g.recs.push_back(r);
        char qname[16];
        std::snprintf(qname, sizeof qname, "Q%d", g.qDone + 1);
        if (FILE* f = std::fopen("vexsim_stats.jsonl", "a")) {
            static const int RPV[4] = {200, 360, 450, 600};
            static const char* LN[4] = {"cascade", "DR4B", "DR6B", "clawbot"};
            static const char* RD[7] = {"QF1", "QF2", "QF3", "QF4", "SF1", "SF2", "F"};
            std::fprintf(f, "{\"bracket\":%ld,\"match\":\"%s\",\"seconds\":%d,\"redAlliance\":%d,\"blueAlliance\":%d,\"redScore\":%d,\"blueScore\":%d,\"winner\":\"%s\",\"redMinor\":%d,\"blueMinor\":%d,\"redDQ\":%s,\"blueDQ\":%s,\"midRed\":%d,\"midBlue\":%d,\"teams\":[",
                         g.bracketId, qual ? qname : RD[std::clamp(g.bracketCur, 0, 6)], (int)g_matchLen, qual ? 0 : m.a + 1, qual ? 0 : m.b + 1, g.red, g.blue, tie ? "tie" : m.winner == m.a ? "red" : "blue",
                         g.minors[0], g.minors[1], g.dq[0] ? "true" : "false", g.dq[1] ? "true" : "false", g.midRed, g.midBlue);
            for (int i = 0; i < 4; ++i) {
                const TeamMatchStat& x = r.t[i];
                std::fprintf(f, "%s{\"team\":\"%s\",\"alliance\":%d,\"color\":\"%s\",\"rpm\":%d,\"lift\":\"%s\",\"wheels\":\"%s\",\"scores\":\"%s\",\"focus\":\"%s\",\"placed\":%d,\"loaded\":%d,\"toggles\":%d,\"points\":%d}",
                             i ? "," : "", x.num.c_str(), qual ? 0 : x.alliance + 1, x.red ? "red" : "blue", RPV[std::clamp(x.rpm, 0, 3)], LN[std::clamp(x.lift, 0, 3)],
                             x.omni ? "all omni" : "traction mid", x.yel ? "yellow" : "own color", focusName(x.focus), x.placed, x.loaded, x.toggles, x.points);
            }
            std::fprintf(f, "]}\n");
            std::fclose(f);
        }
    }
    if (qual) {  // qualification: win points and score for each of the four teams
        const std::array<int, 4>& sc = g.qSched[g.qDone];
        for (int ti = 0; ti < 4; ++ti) {
            const bool red = ti < 2;
            QStat& q = g.qs[sc[ti]];
            q.pts += red ? g.red : g.blue;
            if (tie) ++q.t; else if (red == (m.winner == m.a)) ++q.w; else ++q.l;
        }
        if ((int)g.qRes.size() <= g.qDone) g.qRes.resize(g.qDone + 1, std::array<int, 3>{{-1, -1, -1}});
        g.qRes[g.qDone] = {{g.red, g.blue, tie ? 2 : (m.winner == m.a ? 0 : 1)}};
        ++g.qDone;
        return;
    }
    const int k = g.bracketCur;
    const int parent = k < 4 ? 4 + k / 2 : k < 6 ? 6 : -1;
    if (parent < 0) return;
    const int idx = k < 4 ? k : k - 4;
    BMatch& p = g.bM[parent];
    (idx % 2 == 0 ? p.a : p.b) = m.winner;
    if (p.a >= 0 && p.b >= 0 && p.a > p.b) std::swap(p.a, p.b);  // the better seed plays red
}

// ── Start screen (MATCH SETUP): settings picked with the mouse before PLAY ──
// Laid out in the window's logical pixels (1740 x 800); the renderer draws these widgets and the
// input handler clicks them, so both use this one list.
struct DashWidget {
    int x, y, w, h;      // box (w == 0: text only, drawn from x, y)
    int id;              // what a click changes (-1: nothing)
    std::string text;
    int kind;            // 0 button, 1 text
    int col;             // text: 0 white, 1 red, 2 blue, 3 grey, 4 gold
    int scale;
    bool on;             // can be clicked (dim when not)
    bool sel;            // the chosen one of a set
    bool hit(int px, int py) const { return w > 0 && px >= x && px < x + w && py >= y && py < y + h; }
};
enum { D_BOTS = 100, D_DRIVER = 200, D_LEVEL = 300, D_FOCUS = 400, D_WHEELS = 500, D_SCORE = 501, D_TIMER = 502, D_STARTT = 504, D_WHEELBOT = 560, D_PLAY = 600,
       D_BRACKET = 601, D_SIMONE = 602, D_BSIM = 700, D_BWATCH = 701, D_BSIMALL = 702, D_BNEW = 703, D_BBACK = 704,
       D_BLEVEL = 800, D_BFOCUS = 900, D_BRPM = 1000, D_BLIFT = 1100, D_BWHEEL = 1200, D_BYEL = 1300, D_BINFO = 1400, D_IBACK = 1450, D_LEADER = 1460, D_LBACK = 1461,  // + alliance * 2 + team
       D_BSTART = 705, D_BRANDOM = 706, D_BEDIT = 707, D_PSIM = 708, D_PWATCH = 709, D_PBACK = 710, D_LOADS = 503, D_SCOREBOT = 520, D_YOURWHEELS = 530, D_LIFTBOT = 550, D_RPMBOT = 540 };
enum { D_QSIM = 711, D_QWATCH = 712, D_QALL = 713, D_QFULL = 714, D_QALLI = 715, D_QPAGE = 716, D_TPAGE = 717, D_QVIEW = 718, D_QMPAGE = 719, D_LTAB0 = 720, D_LTAB1 = 721, D_QSPAGE = 722, D_QSIMN = 723, D_QCOUNT = 724, D_LPAGE = 725, D_TRPM = 2000, D_TLIFT = 2100, D_TWHEEL = 2200 };
// Button colour codes (DashWidget::col on a button): every hardware option has its own colour, see the palette in Renderer.cpp
inline int colRpm(int v) { return 1 + std::clamp(v, 0, 3); }
inline int colLift(int v) { return 5 + std::clamp(v, 0, 3); }
inline int colWheels(bool omni) { return omni ? 10 : 9; }
inline int colScores(bool yel) { return yel ? 12 : 11; }
inline const char* levelName(int l) {
    static const char* n[4] = {"ROOKIE", "REGIONAL", "STATE", "WORLDS"};
    return n[std::clamp(l, 0, 3)];
}
inline std::vector<DashWidget> bracketWidgets(const Game& g);
inline std::vector<DashWidget> teamsWidgets(const Game& g);
inline std::vector<DashWidget> previewWidgets(const Game& g);
inline std::vector<DashWidget> infoWidgets(const Game& g);
inline std::vector<DashWidget> leaderWidgets(const Game& g);
inline std::vector<DashWidget> qualWidgets(const Game& g);
inline std::vector<DashWidget> dashWidgets(const Game& g) {
    if (g.dashPage == 1) return bracketWidgets(g);
    if (g.dashPage == 2) return teamsWidgets(g);
    if (g.dashPage == 3) return previewWidgets(g);
    if (g.dashPage == 4) return infoWidgets(g);
    if (g.dashPage == 5) return leaderWidgets(g);
    if (g.dashPage == 6) return qualWidgets(g);
    const SimSettings& st = g.set;
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    auto button = [&](int x, int y, int bw, int bh, int id, const std::string& t, bool on, bool sel, int scale = 2) {
        w.push_back({x, y, bw, bh, id, t, 0, 0, scale, on, sel});
    };
    text(X, Y + 18, "VEX V5 OVERRIDE SIM", 1, 4, W);
    text(X, Y + 70, "MATCH SETUP", 3, 2, W);
    // Which robots play
    text(X + 40, Y + 140, "OTHER ROBOTS", 0, 2);
    static const char* bots[BOT_CHOICE_N] = {"NONE", "PARTNER", "1 OPP", "2 OPP", "PARTNER+1", "ALL 3"};
    for (int k = 0; k < BOT_CHOICE_N; ++k)
        button(X + 300 + k * 163, Y + 128, 155, 42, D_BOTS + k, bots[k], true, st.bots == k);
    // One column per robot: driver and focus (every CPU robot plays at the same level)
    const int mask = BOT_CHOICES[std::clamp(st.bots, 0, BOT_CHOICE_N - 1)];
    static const char* heads[4] = {"YOUR ROBOT", "PARTNER", "OPPONENT 1", "OPPONENT 2"};
    static const char* plates[4] = {"41658F", "41658F", "41658C", "41658D"};
    text(X + 40, Y + 269, "DRIVER", 0, 2);
    text(X + 40, Y + 324, "FOCUS", 0, 2);
    text(X + 40, Y + 366, "SCORES", 0, 2);
    text(X + 40, Y + 408, "LIFT", 0, 2);
    for (int c = 0; c < 4; ++c) {
        const int cx = X + 300 + c * 245, cw = 230;
        const bool active = c == 0 || ((mask >> (c - 1)) & 1);
        const bool cpu = c > 0 || st.pilot > 0;
        text(cx, Y + 196, heads[c], active ? (c < 2 ? 1 : 2) : 3, 2, cw);
        text(cx, Y + 222, active ? plates[c] : "NOT PLAYING", 3, 1, cw);
        if (c == 0) button(cx, Y + 255, cw, 44, D_DRIVER, st.pilot > 0 ? "CPU DRIVES" : "YOU DRIVE", true, st.pilot > 0);
        else        button(cx, Y + 255, cw, 44, -1, "CPU", false, false);
        const int foc = c == 0 ? st.focus[3] : st.focus[c - 1];
        button(cx, Y + 310, cw, 44, D_FOCUS + c, focusName(foc), active && cpu, false);
        {
            const int ri = c == 0 ? 3 : c - 1;
            const bool yv = st.yel[ri] != 0;
            button(cx, Y + 358, 112, 34, D_SCOREBOT + c, yv ? "YELLOW" : (c < 2 ? "RED" : "BLUE"), active && cpu, yv, 2);
            w.back().col = yv ? 12 : 11;
            static const char* rn[4] = {"200 RPM", "360 RPM", "450 RPM", "600 RPM"};
            button(cx + 118, Y + 358, 112, 34, D_RPMBOT + c, rn[std::clamp(st.rpmBot[ri], 0, 3)], active, st.rpmBot[ri] != 2, 2);
            w.back().col = colRpm(st.rpmBot[ri]);
            static const char* ln[4] = {"CASCADE", "DR4B", "DR6B", "CLAWBOT"};
            button(cx, Y + 398, cw, 34, D_LIFTBOT + c, ln[std::clamp(st.liftBot[ri], 0, 3)], active, st.liftBot[ri] != 2, 2);
            w.back().col = colLift(st.liftBot[ri]);
        }
    }
    // Options
    text(X + 40, Y + 496, "SCOREBOARD", 0, 2);
    button(X + 300, Y + 482, 130, 44, D_SCORE, (st.hud & 1) ? "ON" : "OFF", true, (st.hud & 1) != 0);
    text(X + 460, Y + 496, "TIMER", 0, 2);
    button(X + 560, Y + 482, 130, 44, D_TIMER, (st.hud & 2) ? "ON" : "OFF", true, (st.hud & 2) != 0);
    text(X + 740, Y + 496, "MATCH LOADS", 0, 2);
    button(X + 1000, Y + 482, 230, 44, D_LOADS, st.infiniteLoads ? "UNLIMITED" : "10 EACH", true, st.infiniteLoads);
    text(X + 40, Y + 548, "START TIME", 0, 2);
    { const int ss = st.startSecs; button(X + 300, Y + 534, 160, 44, D_STARTT, g.timeEdit ? g.timeBuf + "_" : std::to_string(ss / 60) + ":" + (ss % 60 < 10 ? "0" : "") + std::to_string(ss % 60), true, g.timeEdit || ss != 105); }
    text(X + 490, Y + 548, g.timeEdit ? "TYPE MINUTES:SECONDS (3:45 OR 345)   ENTER OK   ESC CANCEL" : "CLICK TO TYPE A TIME (0:01 TO 10:00); RIGHT CLICK OR LEFT/RIGHT KEYS STEP   F8 ENDS A MATCH   F9 JUMPS TO 15 S LEFT", g.timeEdit ? 0 : 3, 1);

    if (g.lastRed >= 0) {
        const char* who = g.lastRed > g.lastBlue ? "RED WON" : g.lastBlue > g.lastRed ? "BLUE WON" : "TIE";
        text(X, Y + 590, std::string(g.lastSimulated ? "SIMULATED MATCH: RED " : "LAST MATCH: RED ") + std::to_string(g.lastRed) + " - " + std::to_string(g.lastBlue) +
                         " BLUE   " + who, 4, 2, W);
    }
    button(X + W / 2 - 590, Y + 616, 360, 90, D_PLAY, "PLAY", true, true, 6);
    button(X + W / 2 - 180, Y + 616, 360, 90, D_SIMONE, "SIM MATCH", true, false, 3);
    button(X + W / 2 + 230, Y + 616, 360, 90, D_BRACKET, "TOURNAMENT", true, false, 3);
    text(X, Y + 716, "LEFT CLICK CHANGES   RIGHT CLICK GOES BACK   ARROW KEYS PICK   ENTER PLAYS", 3, 2, W);
    return w;
}
// The start screen's widgets that can be picked (with the keyboard), in order; PLAY last
inline std::vector<int> dashPickable(const std::vector<DashWidget>& w) {
    std::vector<int> v;
    for (int i = 0; i < (int)w.size(); ++i)
        if (w[i].kind == 0 && w[i].on && w[i].id >= 0 && !(w[i].id > D_BOTS && w[i].id < D_BOTS + BOT_CHOICE_N)) v.push_back(i);
    std::stable_sort(v.begin(), v.end(), [&](int a, int b) { return w[a].y != w[b].y ? w[a].y < w[b].y : w[a].x < w[b].x; });  // row by row
    return v;  // OTHER ROBOTS counts once (its first button); Left / Right step through its choices
}
// The tournament page: the 8-alliance bracket in three columns, and the buttons
inline std::vector<DashWidget> bracketWidgets(const Game& g) {
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    auto button = [&](int x, int y, int bw, int bh, int id, const std::string& t, bool on, bool sel, int scale = 2) {
        w.push_back({x, y, bw, bh, id, t, 0, 0, scale, on, sel});
    };
    const int next = bracketNext(g);
    text(X, Y + 14, "TOURNAMENT", 1, 3, W);
    static const char* rounds[3] = {"QUARTERFINALS", "SEMIFINALS", "FINAL"};
    const int colX[3] = {X + 10, X + 360, X + 710}, colW = 320, boxH = 92, pitch = 124;
    double ys[BR_MATCHES];
    for (int k = 0; k < 4; ++k) ys[k] = Y + 100 + k * pitch;
    ys[4] = (ys[0] + ys[1]) / 2;
    ys[5] = (ys[2] + ys[3]) / 2;
    ys[6] = (ys[4] + ys[5]) / 2;
    for (int c = 0; c < 3; ++c) text(colX[c], Y + 70, rounds[c], 3, 2, colW);
    for (int k = 0; k < BR_MATCHES; ++k) {
        const BMatch& m = g.bM[k];
        const int c = k < 4 ? 0 : k < 6 ? 1 : 2;
        const int bx = colX[c], by = (int)ys[k];
        w.push_back({bx, by, colW, boxH, -1, "", 2, 0, 1, true, k == next});
        for (int sd = 0; sd < 2 && m.winner >= 0; ++sd) {  // the winner's row is filled with its alliance colour, the loser's outlined
            const int al2 = sd == 0 ? m.a : m.b;
            w.push_back({bx + 70, by + 5 + sd * 43, colW - 75, 38, -1, "", 2, m.winner == al2 ? (sd == 0 ? 5 : 6) : (sd == 0 ? 7 : 8), 1, true, false});
        }
        for (int side = 0; side < 2; ++side) {  // red alliance on top, blue below
            const int al = side == 0 ? m.a : m.b;
            const int ly = by + 5 + side * 43;
            if (al < 0) { text(bx + 20, ly + 10, "-", 3, 3); continue; }
            const BAlliance& A = g.bAl[al];
            const bool won = m.winner == al, lost = m.winner >= 0 && !won;
            // The alliance number in a badge of the colour it plays (dark once it has lost)
            w.push_back({bx + 5, ly, 62, 38, D_BINFO + al, "#" + std::to_string(al + 1), 3, (side == 0 ? 1 : 2) + (lost ? 4 : 0), 3, true, false});
            for (int t = 0; t < 2; ++t) {
                text(bx + 76, ly + 4 + t * 17, A.t[t].num, lost ? 3 : (side == 0 ? 1 : 2), 2);
            }
            w.push_back({bx + 70, ly, colW - 75, 38, D_BINFO + al, "", 1, 0, 2, true, false});  // the whole row opens the alliance page
            if (m.winner >= 0) text(bx + colW - 64, ly + 8, std::to_string(side == 0 ? m.ra : m.rb), lost ? 3 : 4, 3);
        }
    }
    const int champ = g.bM[6].winner;
    if (champ >= 0) {
        const BAlliance& A = g.bAl[champ];
        text(colX[2], (int)ys[6] - 62, "CHAMPIONS", 4, 3, colW);
        text(colX[2], (int)ys[6] - 30, "#" + std::to_string(champ + 1) + " " + A.t[0].num + " " + A.t[1].num, 4, 2, colW);
    }
    const int bxp = X + 1060;
    if (next >= 0) text(bxp, Y + 70, "NEXT: #" + std::to_string(g.bM[next].a + 1) + " VS #" + std::to_string(g.bM[next].b + 1), 0, 2);
    const bool can = next >= 0;
    button(bxp, Y + 100, 230, 46, D_BSIM, "SIM NEXT MATCH", can, false);
    button(bxp, Y + 158, 230, 46, D_BWATCH, "WATCH NEXT MATCH", can, false);
    button(bxp, Y + 216, 230, 46, D_BSIMALL, "SIM ALL", can, false);
    button(bxp, Y + 274, 230, 46, D_BEDIT, "TEAMS AND ROBOTS", true, false);
    button(bxp, Y + 332, 230, 46, D_BNEW, "NEW BRACKET", true, false);
    button(bxp, Y + 448, 230, 46, D_LEADER, "LEADERBOARDS", true, false);
    button(bxp, Y + 506, 230, 46, D_QPAGE, "QUALIFICATION", true, false);
    button(bxp, Y + 390, 230, 46, D_BBACK, "BACK", true, false);
    text(X + 20, Y + 672, "CLICK AN ALLIANCE TO SEE ITS HARDWARE AND STATS.  EVERY MATCH IS SAVED TO VEXSIM_STATS.JSONL", 3, 1);
    text(X + 20, Y + 690, "SIM PLAYS A MATCH OUT AT FULL SPEED, NOT SHOWN", 3, 1);
    return w;
}

// Qualification page: standings (20 teams a page) and match results side by side, and the buttons to play the 50 matches
inline std::vector<DashWidget> qualWidgets(const Game& g) {
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    auto button = [&](int x, int y, int bw, int bh, int id, const std::string& t, bool on, bool sel, int scale = 2) {
        w.push_back({x, y, bw, bh, id, t, 0, 0, scale, on, sel});
    };
    const int total = qualTotal(g), done = g.qDone;
    text(X, Y + 14, "QUALIFICATION", 1, 3, W);
    text(X, Y + 54, done >= total ? "ALL " + std::to_string(total) + " QUALIFICATION MATCHES PLAYED" : "MATCH " + std::to_string(done + 1) + " OF " + std::to_string(total) + "   TOP 16 GO TO THE ELIMINATION ROUNDS", 3, 2, W);
    // Light tables like RobotEvents: white cards, dark text, the winning alliance's cell tinted pink or blue
    const std::vector<int> r = qualRanking(g);
    const int sp = std::clamp(g.qSPage, 0, 1);
    {
        const int px = X + 10, py = Y + 86;
        w.push_back({px, py, 430, 572, -1, "", 2, 9, 1, true, false});
        text(px + 14, py + 12, "RANK", 6, 1); text(px + 74, py + 12, "TEAM", 6, 1); text(px + 160, py + 12, "W-L-T", 6, 1); text(px + 252, py + 12, "WP / PTS", 6, 1);
        for (int j = 0; j < 20; ++j) {
            const int i = sp * 20 + j, ti = r[i];
            const QStat& q = g.qs[ti];
            const int ry = py + 32 + j * 27;
            const int c = i >= 16 ? 6 : 5;
            char b[48];
            w.push_back({px, ry + 26, 430, i == 15 ? 2 : 1, -1, "", 2, i == 15 ? 14 : 12, 1, true, false});  // row line; the elimination cutoff is amber
            text(px + 14, ry + 6, std::to_string(i + 1), i < 8 ? 9 : c, 2);
            text(px + 74, ry + 6, g.teams[ti].num, c, 2);
            std::snprintf(b, sizeof b, "%d-%d-%d", q.w, q.l, q.t); text(px + 160, ry + 6, b, c, 2);
            std::snprintf(b, sizeof b, "%d / %d", q.wp(), q.pts); text(px + 252, ry + 6, b, c, 2);
        }
    }
    {
        const int px = X + 446, py = Y + 86;
        const int pages = std::max(1, (total + 11) / 12), pg = g.qMPage < 0 ? std::clamp((done - 1) / 12, 0, pages - 1) : std::clamp(g.qMPage, 0, pages - 1);
        w.push_back({px, py, 574, 572, -1, "", 2, 9, 1, true, false});
        text(px + 14, py + 12, "MATCH", 6, 1); text(px + 142, py + 12, "RED ALLIANCE", 7, 1); text(px + 366, py + 12, "BLUE ALLIANCE", 8, 1);
        text(px + 574 - 14 - 90, py + 12, "PAGE " + std::to_string(pg + 1) + "/" + std::to_string(pages), 6, 1);
        for (int j = 0; j < 12; ++j) {
            const int mi = pg * 12 + j;
            if (mi >= total) break;
            const int y = py + 30 + j * 45;
            const std::array<int, 4>& sc = g.qSched[mi];
            const std::array<int, 3> rs = mi < (int)g.qRes.size() ? g.qRes[mi] : std::array<int, 3>{{-1, -1, -1}};
            const bool played = rs[2] >= 0, nextM = mi == done && g.stage == 1;
            const int rx = px + 132, bx = px + 356, gw = 214;
            if (rs[2] == 0) w.push_back({rx, y, gw, 44, -1, "", 2, 10, 1, true, false});
            else if (rs[2] == 1) w.push_back({bx, y, gw, 44, -1, "", 2, 11, 1, true, false});
            else if (rs[2] == 2) { w.push_back({rx, y, gw, 44, -1, "", 2, 13, 1, true, false}); w.push_back({bx, y, gw, 44, -1, "", 2, 13, 1, true, false}); }
            w.push_back({px, y + 44, 574, 1, -1, "", 2, 12, 1, true, false});
            if (nextM) w.push_back({px, y, 4, 44, -1, "", 2, 14, 1, true, false});
            const int mins = 9 * 60 + mi * 7;
            char tb[48];
            std::snprintf(tb, sizeof tb, "Round %d  %d:%02d %s", mi / 10 + 1, (mins / 60 - 1) % 12 + 1, mins % 60, mins / 60 >= 12 ? "PM" : "AM");
            text(px + 14, y + 8, "Qualifier #" + std::to_string(mi + 1), 5, 1);
            text(px + 14, y + 24, nextM ? "NEXT MATCH" : tb, nextM ? 9 : 6, 1);
            text(rx + 12, y + 13, g.teams[sc[0]].num, 5, 2); text(rx + 84, y + 13, g.teams[sc[1]].num, 5, 2);
            text(bx + 12, y + 13, g.teams[sc[2]].num, 5, 2); text(bx + 84, y + 13, g.teams[sc[3]].num, 5, 2);
            if (played) {
                const std::string a = std::to_string(rs[0]), b2 = std::to_string(rs[1]);
                text(rx + gw - 12 - 10 * (int)a.size(), y + 13, a, 5, 2);
                text(bx + gw - 12 - 10 * (int)b2.size(), y + 13, b2, 5, 2);
            }
        }
    }
    text(X + 10, Y + 664, "RESULTS: THE WINNING ALLIANCE IS TINTED RED OR BLUE, YELLOW IS A TIE.   STANDINGS: AMBER RANKS ARE CAPTAINS (1-8), THE AMBER LINE IS THE ELIMINATION CUTOFF (16), WP = WIN POINTS.", 3, 1);
    const int bxp = X + 1060;
    if (done < total && g.stage == 1) {
        const std::array<int, 4>& sc = g.qSched[done];
        text(bxp, Y + 86, "NEXT RED  " + g.teams[sc[0]].num + " " + g.teams[sc[1]].num, 1, 1);
        text(bxp, Y + 104, "NEXT BLUE " + g.teams[sc[2]].num + " " + g.teams[sc[3]].num, 2, 1);
        button(bxp, Y + 126, 230, 40, D_QSIM, "SIM NEXT MATCH", true, false);
        button(bxp, Y + 172, 230, 40, D_QWATCH, "WATCH NEXT MATCH", true, false);
        button(bxp, Y + 218, 62, 40, D_QCOUNT, g.qCountEdit ? g.qCountBuf + "_" : std::to_string(g.qCount), true, g.qCountEdit);
        button(bxp + 68, Y + 218, 162, 40, D_QSIMN, "SIM NEXT " + std::to_string(std::min(g.qCount, total - done)), true, false);
        text(bxp, Y + 262, g.qCountEdit ? "TYPE A NUMBER, ENTER TO ACCEPT" : "CLICK THE BOX TO TYPE HOW MANY", 3, 1);
        button(bxp, Y + 278, 230, 40, D_QALL, "SIM ALL QUALS", true, false);
        button(bxp, Y + 324, 230, 40, D_QFULL, "SIM WHOLE EVENT", true, false);
    } else if (g.stage == 1) {
        button(bxp, Y + 126, 230, 62, D_QALLI, "ALLIANCE SELECTION", true, true);
    } else {
        button(bxp, Y + 126, 230, 62, D_PBACK, "BACK TO BRACKET", true, true);
    }
    button(bxp, Y + 374, 230, 40, D_QSPAGE, sp == 0 ? "STANDINGS 21-40" : "STANDINGS 1-20", true, false);
    button(bxp, Y + 418, 230, 40, D_QMPAGE, "NEXT RESULTS PAGE", true, false);
    button(bxp, Y + 466, 230, 40, D_BEDIT, "TEAMS AND ROBOTS", true, false);
    button(bxp, Y + 510, 230, 40, D_LEADER, "LEADERBOARDS", true, false);
    button(bxp, Y + 554, 230, 40, D_BNEW, "NEW TOURNAMENT", true, false);
    button(bxp, Y + 598, 230, 40, D_BBACK, "BACK", true, false);
    return w;
}

// The 40 teams before qualification: each team's drivetrain rpm, lift and wheels (20 a page)
inline std::vector<DashWidget> teamPickWidgets(const Game& g) {
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300, per = 20, pages = QUAL_N / per;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    auto button = [&](int x, int y, int bw, int bh, int id, const std::string& t, bool on, bool sel, int scale = 2) {
        w.push_back({x, y, bw, bh, id, t, 0, 0, scale, on, sel});
    };
    text(X, Y + 14, "TOURNAMENT TEAMS", 1, 3, W);
    text(X, Y + 50, "40 TEAMS PLAY QUALIFICATION MATCHES; THE TOP 16 BECOME 8 ALLIANCES FOR THE ELIMINATION ROUNDS. PICK EACH ROBOT.", 3, 1, W);
    text(X, Y + 68, "PAGE " + std::to_string(g.teamPage + 1) + " OF " + std::to_string(pages), 3, 2, W);
    static const char* rn[4] = {"200 RPM", "360 RPM", "450 RPM", "600 RPM"};
    static const char* ln[4] = {"CASCADE", "DR4B", "DR6B", "CLAWBOT"};
    for (int j = 0; j < per; ++j) {
        const int ti = g.teamPage * per + j;
        const BTeam& tm = g.teams[ti];
        const int x = X + 30 + (j / 10) * 640, y = Y + 94 + (j % 10) * 52;
        text(x, y + 14, std::to_string(ti + 1) + ".", 3, 2);
        text(x + 56, y + 14, tm.num, 0, 2);
        button(x + 170, y + 4, 150, 40, D_TRPM + ti, rn[std::clamp(tm.rpm, 0, 3)], true, tm.rpm != 2, 2);
        w.back().col = colRpm(tm.rpm);
        button(x + 330, y + 4, 170, 40, D_TLIFT + ti, ln[std::clamp(tm.lift, 0, 3)], true, tm.lift != 2, 2);
        w.back().col = colLift(tm.lift);
    }
    button(X + 30, Y + 650, 200, 50, D_BBACK, "BACK", true, false, 2);
    button(X + 250, Y + 650, 260, 50, D_TPAGE, "NEXT PAGE", true, false, 2);
    button(X + 530, Y + 650, 260, 50, D_BRANDOM, "RANDOMIZE", true, false, 2);
    button(X + 810, Y + 650, 400, 50, D_BSTART, "START QUALIFICATION", true, true, 2);
    return w;
}

// Tournament teams page: every team's focus, set before the bracket starts (all teams play at the same level)
inline std::vector<DashWidget> teamsWidgets(const Game& g) {
    if (g.stage < 2) return teamPickWidgets(g);
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    auto button = [&](int x, int y, int bw, int bh, int id, const std::string& t, bool on, bool sel, int scale = 2) {
        w.push_back({x, y, bw, bh, id, t, 0, 0, scale, on, sel});
    };
    text(X, Y + 14, "TOURNAMENT TEAMS", 1, 3, W);
    text(X, Y + 56, "PICK EACH TEAM'S DRIVETRAIN RPM, LIFT AND WHEELS. FOCUS IS PICKED BEFORE EACH MATCH.", 3, 2, W);
    for (int i = 0; i < BR_ALLIANCES; ++i) {
        const int col = i / 4, row = i % 4;
        const int x = X + 30 + col * 640, y = Y + 100 + row * 130;
        w.push_back({x, y + 6, 74, 100, D_BINFO + i, std::to_string(i + 1), 3, i == 0 ? 1 : 0, 4, true, false});  // alliance number: click for its hardware
        for (int t = 0; t < 2; ++t) {
            const BTeam& tm = g.bAl[i].t[t];
            text(x + 96, y + 22 + t * 50, tm.num, 0, 2);
            static const char* rn[4] = {"200 RPM", "360 RPM", "450 RPM", "600 RPM"};
            static const char* ln[4] = {"CASCADE", "DR4B", "DR6B", "CLAWBOT"};
            button(x + 190, y + 10 + t * 50, 112, 40, D_BRPM + i * 2 + t, rn[std::clamp(tm.rpm, 0, 3)], true, tm.rpm != 2, 2);
            w.back().col = colRpm(tm.rpm);
            button(x + 308, y + 10 + t * 50, 130, 40, D_BLIFT + i * 2 + t, ln[std::clamp(tm.lift, 0, 3)], true, tm.lift != 2, 2);
            w.back().col = colLift(tm.lift);
        }
    }
    button(X + 30, Y + 650, 200, 50, D_BBACK, "BACK", true, false, 2);
    button(X + W / 2 - 320, Y + 650, 300, 50, D_BRANDOM, "RANDOMIZE", true, false, 2);
    button(X + W / 2 + 20, Y + 650, 300, 50, D_BSTART, "START BRACKET", true, true, 2);
    return w;
}

// Match preview: the two alliances and each team's focus for this match
inline std::vector<DashWidget> previewWidgets(const Game& g) {
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    auto button = [&](int x, int y, int bw, int bh, int id, const std::string& t, bool on, bool sel, int scale = 2) {
        w.push_back({x, y, bw, bh, id, t, 0, 0, scale, on, sel});
    };
    const int k = bracketNext(g);
    if (k < 0) { button(X + W / 2 - 150, Y + 650, 300, 50, D_PBACK, "BACK", true, false); return w; }
    const BMatch& m = g.bM[k];
    const std::string round = k < 4 ? "QUARTERFINAL " + std::to_string(k + 1) : k < 6 ? "SEMIFINAL " + std::to_string(k - 3) : std::string("FINAL");
    text(X, Y + 14, round, 4, 3, W);
    text(X, Y + 56, "PICK EACH TEAM'S FOCUS AND WHAT IT SCORES FOR THIS MATCH", 3, 2, W);
    for (int side = 0; side < 2; ++side) {
        const int al = side == 0 ? m.a : m.b;
        const BAlliance& A = g.bAl[al];
        const int px = X + 50 + side * 620, py = Y + 110;
        w.push_back({px, py, 580, 400, -1, "", 2, 0, 1, true, false});  // panel
        w.push_back({px + 20, py + 20, 90, 70, D_BINFO + al, std::to_string(al + 1), 3, side == 0 ? 1 : 2, 4, true, false});
        text(px + 130, py + 30, side == 0 ? "RED ALLIANCE" : "BLUE ALLIANCE", side == 0 ? 1 : 2, 3);
        for (int t = 0; t < 2; ++t) {
            const BTeam& tm = A.t[t];
            const int ty = py + 130 + t * 130;
            text(px + 30, ty, tm.num, 0, 3);
            text(px + 30, ty + 52, "FOCUS", 3, 2);
            button(px + 140, ty + 44, 300, 38, D_BFOCUS + al * 2 + t, focusName(tm.focus), true, true, 2);
            text(px + 30, ty + 96, "SCORES", 3, 2);
            button(px + 140, ty + 88, 300, 32, D_BYEL + al * 2 + t, tm.yel ? "YELLOW" : (side == 0 ? "OWN COLOR (RED)" : "OWN COLOR (BLUE)"), true, tm.yel, 2);
            w.back().col = colScores(tm.yel);
            static const char* rn[4] = {"200 RPM", "360 RPM", "450 RPM", "600 RPM"};
            static const char* ln[4] = {"CASCADE", "DR4B", "DR6B", "CLAWBOT"};
            text(px + 300, ty + 4, std::string(rn[std::clamp(tm.rpm, 0, 3)]) + "  " + ln[std::clamp(tm.lift, 0, 3)], 3, 2);
        }
    }
    text(X + 50, Y + 536, "SCORING (80 PERCENT OF THE TIME): THE LOADER-TO-GOAL CYCLE ON ITS OWN HALF OF THE FIELD; THE OTHER 20 PERCENT IT DEFENDS.", 3, 1);
    text(X + 50, Y + 554, "DEFENSE/TOGGLE (80 PERCENT): TURNS THE TOGGLES, ELSE SHADOWS AND BLOCKS OPPONENTS; THE OTHER 20 PERCENT IT SCORES.", 3, 1);
    text(X + 50, Y + 572, "TOGGLE/SCORING (80 PERCENT): TURNS A TOGGLE WHENEVER ONE IS WORTH IT, OTHERWISE SCORES PIECES; THE OTHER 20 PERCENT IT DEFENDS.", 3, 1);
    text(X + 50, Y + 590, "THE TWO ROBOTS OF AN ALLIANCE SPLIT THE FIELD: EACH WORKS ITS OWN HALF.", 3, 1);
    button(X + 50, Y + 640, 200, 60, D_PBACK, "BACK", true, false, 2);
    button(X + W / 2 - 320, Y + 630, 300, 70, D_PSIM, "SIMULATE", true, g.previewFast, 3);
    button(X + W / 2 + 20, Y + 630, 300, 70, D_PWATCH, "WATCH", true, !g.previewFast, 3);
    return w;
}

// Leaderboards: teams ranked by wins, then average alliance score. TOURNAMENT is this bracket's matches; ALL-TIME is every match in vexsim_stats.jsonl.
struct TeamAgg { std::string num; int mp = 0, wins = 0, ties = 0, pts = 0, placed = 0, loaded = 0, toggles = 0, points = 0; };
inline std::vector<TeamAgg> aggregateRecs(const std::vector<MatchRec>& recs) {
    std::vector<TeamAgg> v;
    for (const MatchRec& r : recs)
        for (const TeamMatchStat& x : r.t) {
            TeamAgg* a = nullptr;
            for (TeamAgg& e : v) if (e.num == x.num) { a = &e; break; }
            if (!a) { v.emplace_back(); a = &v.back(); a->num = x.num; }
            ++a->mp;
            if (r.winnerAl < 0) ++a->ties; else if (r.winnerAl == x.alliance) ++a->wins;
            a->pts += x.red ? r.redScore : r.blueScore; a->placed += x.placed; a->loaded += x.loaded; a->toggles += x.toggles; a->points += x.points;
        }
    std::stable_sort(v.begin(), v.end(), [](const TeamAgg& a, const TeamAgg& b) {
        if (a.wins != b.wins) return a.wins > b.wins;
        const double pa = a.mp ? (double)a.pts / a.mp : 0.0, pb = b.mp ? (double)b.pts / b.mp : 0.0;
        return pa > pb;
    });
    return v;
}
// Reads vexsim_stats.jsonl (one match a line, as bracketRecord writes it)
inline void loadAllTime(Game& g) {
    g.allRecs.clear();
    FILE* f = std::fopen("vexsim_stats.jsonl", "r");
    if (!f) return;
    std::string line;
    int ch;
    auto numAfter = [](const std::string& l, const std::string& key, size_t from, size_t* at) -> int {
        const size_t k = l.find("\"" + key + "\":", from);
        if (k == std::string::npos) { if (at) *at = std::string::npos; return 0; }
        const size_t v = k + key.size() + 3;
        if (at) *at = v;
        return std::atoi(l.c_str() + v);
    };
    auto strAfter = [](const std::string& l, const std::string& key, size_t from) -> std::string {
        const size_t k = l.find("\"" + key + "\":\"", from);
        if (k == std::string::npos) return "";
        const size_t v = k + key.size() + 4, e = l.find('"', v);
        return e == std::string::npos ? "" : l.substr(v, e - v);
    };
    auto parse = [&](const std::string& l) {
        if (l.size() < 20) return;
        MatchRec r;
        r.redScore = numAfter(l, "redScore", 0, nullptr);
        r.blueScore = numAfter(l, "blueScore", 0, nullptr);
        { const std::string wn = strAfter(l, "winner", 0); r.winnerAl = wn == "red" ? 0 : wn == "tie" ? -1 : 1; }  // here an "alliance" is the side: 0 red, 1 blue
        size_t pos = l.find("\"teams\":");
        if (pos == std::string::npos) return;
        for (int i = 0; i < 4; ++i) {
            const size_t t = l.find("{\"team\":", pos);
            if (t == std::string::npos) return;
            TeamMatchStat& x = r.t[i];
            x.num = strAfter(l, "team", t);
            x.red = strAfter(l, "color", t) == "red";
            x.alliance = x.red ? 0 : 1;
            x.placed = numAfter(l, "placed", t, nullptr);
            x.loaded = numAfter(l, "loaded", t, nullptr);
            x.toggles = numAfter(l, "toggles", t, nullptr);
            x.points = numAfter(l, "points", t, nullptr);
            pos = t + 5;
        }
        g.allRecs.push_back(r);
    };
    while ((ch = std::fgetc(f)) != EOF) {
        if (ch == '\n') { parse(line); line.clear(); } else line.push_back((char)ch);
    }
    parse(line);
    std::fclose(f);
}
inline std::vector<DashWidget> leaderWidgets(const Game& g) {
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    text(X, Y + 14, "LEADERBOARDS", 4, 3, W);
    w.push_back({X + 30, Y + 6, 140, 40, D_LBACK, "BACK", 0, 0, 2, true, false});
    w.push_back({X + 400, Y + 52, 230, 40, D_LTAB0, "TOURNAMENT", 0, 0, 2, true, g.lbMode == 0});
    w.push_back({X + 640, Y + 52, 230, 40, D_LTAB1, "ALL-TIME", 0, 0, 2, true, g.lbMode == 1});
    std::vector<TeamAgg> v = aggregateRecs(g.lbMode == 0 ? g.recs : g.allRecs);
    if (g.lbMode == 0)  // this tournament's board lists all 40 teams, the ones that have not played yet last
        for (const BTeam& t : g.teams) {
            bool have = false;
            for (const TeamAgg& a : v) if (a.num == t.num) { have = true; break; }
            if (!have) { TeamAgg a; a.num = t.num; v.push_back(a); }
        }
    else  // the all-time board lists all 60 teams, the ones without saved matches last
        for (const char* pn : TEAM_POOL) {
            bool have = false;
            for (const TeamAgg& a : v) if (a.num == pn) { have = true; break; }
            if (!have) { TeamAgg a; a.num = pn; v.push_back(a); }
        }
    const int mTot = (int)(g.lbMode == 0 ? g.recs.size() : g.allRecs.size());
    text(X + 30, Y + 66, g.lbMode == 0 ? std::to_string(mTot) + " MATCHES THIS TOURNAMENT" : std::to_string(mTot) + " MATCHES SAVED", 3, 2);
    text(X + 880, Y + 66, "WINS, THEN AVERAGE SCORE", 3, 1);
    if (v.size() > 40) w.push_back({X + 1100, Y + 52, 190, 40, D_LPAGE, g.lbPage ? "RANKS 1-40" : "RANKS 41-60", 0, 0, 2, true, false});
    for (int side = 0; side < 2; ++side) {
        const int px = X + 10 + side * 650, py = Y + 104;
        w.push_back({px, py, 640, 556, -1, "", 2, 9, 1, true, false});
        const int hy = py + 12;
        text(px + 14, hy, "RANK", 6, 1); text(px + 74, hy, "TEAM", 6, 1); text(px + 170, hy, "W-L-T", 6, 1); text(px + 270, hy, "AVG", 6, 1);
        text(px + 350, hy, "PLACED", 6, 1); text(px + 440, hy, "TOGGLES", 6, 1); text(px + 540, hy, "POINTS", 6, 1);
        if (v.empty() && side == 0) text(px + 20, hy + 40, g.lbMode == 0 ? "NO MATCHES PLAYED YET" : "NO SAVED MATCHES YET", 6, 2);
        for (int j = 0; j < 20; ++j) {
            const int i = g.lbPage * 40 + side * 20 + j;
            if (i >= (int)v.size()) break;
            const TeamAgg& a = v[i];
            const int ry = hy + 24 + j * 26, col = i == 0 ? 9 : 5;
            char b[48];
            w.push_back({px, ry + 25, 640, 1, -1, "", 2, 12, 1, true, false});
            text(px + 14, ry + 5, std::to_string(i + 1), col, 2);
            text(px + 74, ry + 5, a.num, col, 2);
            std::snprintf(b, sizeof b, "%d-%d-%d", a.wins, a.mp - a.wins - a.ties, a.ties); text(px + 170, ry + 5, b, col, 2);
            std::snprintf(b, sizeof b, "%.1f", a.mp ? (double)a.pts / a.mp : 0.0); text(px + 270, ry + 5, b, col, 2);
            text(px + 350, ry + 5, std::to_string(a.placed), col, 2);
            text(px + 440, ry + 5, std::to_string(a.toggles), col, 2);
            text(px + 540, ry + 5, std::to_string(a.points), col, 2);
        }
    }
    return w;
}

// Alliance page (click an alliance): both of its teams, with the alliance's colour and role, each robot's hardware, records and stats
inline std::vector<DashWidget> infoWidgets(const Game& g) {
    std::vector<DashWidget> w;
    const int X = 220, Y = 40, W = 1300;
    auto text = [&](int x, int y, const std::string& t, int col, int scale, int boxW = 0) {
        w.push_back({x, y, boxW, boxW > 0 ? 30 : 0, -1, t, 1, col, scale, true, false});
    };
    const int al = std::clamp(g.infoAl, 0, BR_ALLIANCES - 1);
    // Which colour this alliance plays: in its next match, else in its last one
    int side = -1, where = -1;
    bool next = false;
    for (int k = 0; k < BR_MATCHES; ++k) {
        const BMatch& m = g.bM[k];
        if (m.a != al && m.b != al) continue;
        side = m.a == al ? 0 : 1; where = k;
        if (m.winner < 0 && m.a >= 0 && m.b >= 0) { next = true; break; }
    }
    static const char* mname[7] = {"QUARTERFINAL 1", "QUARTERFINAL 2", "QUARTERFINAL 3", "QUARTERFINAL 4", "SEMIFINAL 1", "SEMIFINAL 2", "FINAL"};
    const bool champ = g.bM[6].winner == al;
    const int sideCol = side < 0 ? 3 : side == 0 ? 1 : 2;
    text(X, Y + 14, "ALLIANCE " + std::to_string(al + 1), sideCol == 3 ? 4 : sideCol, 3, W);
    const int saveType = g_liftType;
    static const char* rn[4] = {"200", "360", "450", "600"};
    static const double RPV[4] = {200.0, 360.0, 450.0, 600.0};
    static const char* ln[4] = {"CASCADE", "DR4B", "DR6B", "CLAWBOT"};
    const std::vector<int> rank = qualRanking(g);
    struct Agg { int mp = 0, w = 0, l = 0, t = 0, placed = 0, tog = 0, pts = 0; };
    auto aggOf = [](const std::vector<MatchRec>& recs, const std::string& num) {
        Agg a;
        for (const MatchRec& r : recs)
            for (const TeamMatchStat& x : r.t)
                if (x.num == num) {
                    ++a.mp;
                    if (r.winnerAl < 0) ++a.t; else if (r.winnerAl == x.alliance) ++a.w; else ++a.l;
                    a.placed += x.placed; a.tog += x.toggles; a.pts += x.points;
                }
        return a;
    };
    for (int k = 0; k < 2; ++k) {
        const BTeam& t = g.bAl[al].t[k];
        const int px = X + 30 + k * 640, py = Y + 60;
        w.push_back({px, py, 600, 600, -1, "", 2, sideCol, 1, true, false});
        // identification: alliance badge in its colour, team number, role and next match
        int ti = -1, qr = -1;
        for (int i = 0; i < QUAL_N && i < (int)g.teams.size(); ++i) if (g.teams[i].num == t.num) ti = i;
        for (int i = 0; i < (int)rank.size(); ++i) if (rank[i] == ti) qr = i + 1;
        w.push_back({px + 18, py + 20, 78, 62, -1, "#" + std::to_string(al + 1), 3, side < 0 ? 0 : (side == 0 ? 1 : 2) + (!next && !champ ? 4 : 0), 3, true, false});
        text(px + 112, py + 18, t.num, 0, 4);
        text(px + 112, py + 52, (k == 0 ? "CAPTAIN" : "PICK") + std::string("  QUAL RANK ") + (qr > 0 ? std::to_string(qr) : "-"), 3, 2);
        std::string st; int stc = 3;
        if (champ) { st = "CHAMPIONS"; stc = 4; }
        else if (next) { st = std::string(side == 0 ? "RED" : "BLUE") + " ALLIANCE IN THE " + mname[where]; stc = side == 0 ? 1 : 2; }
        else if (where >= 0) { st = std::string("OUT IN THE ") + mname[where] + " (WAS " + (side == 0 ? "RED" : "BLUE") + ")"; }
        text(px + 112, py + 74, st, stc, 2);
        const int rp = std::clamp(t.rpm, 0, 3), lf = std::clamp(t.lift, 0, 3);
        const double top = M_PI * 3.25 * RPV[rp] / 60.0;
        setLiftType(lf);
        const double reach = dr4bMaxLiftIn(), spd = g_liftSpeed, full = LIFT_MAX_DEG / (840.0 * spd);
        setLiftType(saveType);
        char b[160];
        int y = py + 112;
        auto line = [&](const std::string& head, const std::string& val, int col = 0) {
            text(px + 20, y, head, 3, 2);
            text(px + 250, y, val, col, 2);
            y += 24;
        };
        auto head = [&](const std::string& h) { y += 8; text(px + 20, y, h, 4, 2); y += 26; };
        head("ROBOT");
        line("MOTORS", "4 X 11 W BLUE 600 RPM");
        line("WHEEL SPEED", std::string(rn[rp]) + " RPM");
        std::snprintf(b, sizeof b, "%.1f IN/S", top); line("TOP SPEED", b);
        line("WHEEL TYPE", "4 OMNI + 2 TRACTION");
        line("LIFT TYPE", ln[lf]);
        std::snprintf(b, sizeof b, "%.1f S", full); line("FULL RAISE", b);
        std::snprintf(b, sizeof b, "%.1f IN", reach); line("MAX REACH", b);
        head("RECORD");
        if (ti >= 0) {
            const QStat& q = g.qs[ti];
            std::snprintf(b, sizeof b, "%d-%d-%d   %d WP", q.w, q.l, q.t, q.wp()); line("QUALIFICATION", q.played() ? b : "NONE PLAYED");
        }
        const Agg sea = aggOf(g.allRecs, t.num), evt = aggOf(g.recs, t.num);
        std::snprintf(b, sizeof b, "%d-%d-%d  (%d MATCHES)", sea.w, sea.l, sea.t, sea.mp); line("SEASON", sea.mp ? b : "NO SAVED MATCHES");
        head("STATS");
        text(px + 330, y - 26, "EVENT", 3, 2); text(px + 460, y - 26, "SEASON", 3, 2);
        auto row = [&](const std::string& label, int e, int se) {
            text(px + 20, y, label, 3, 2);
            text(px + 330, y, std::to_string(e), 0, 2);
            text(px + 460, y, std::to_string(se), 0, 2);
            y += 24;
        };
        row("PIECES PLACED", evt.placed, sea.placed);
        row("TOGGLES KEPT", evt.tog, sea.tog);
        row("POINTS SCORED", evt.pts, sea.pts);
        text(px + 20, y + 4, "TOGGLES KEPT: ROLLED BY THIS ROBOT AND STILL ITS COLOUR AT THE BUZZER", 3, 1);
    }
    w.push_back({X + 30, Y + 6, 140, 40, D_IBACK, "BACK", 0, 0, 2, true, false});
    return w;
}

// START TIME typed as minutes and seconds: "3:45" is 3 min 45 s; digits alone are seconds (up to 2 digits) or MMSS ("345"), 1 s to 10:00
inline int parseStartTime(const std::string& s) {
    const size_t c = s.find(':');
    long v;
    if (c != std::string::npos) v = (c == 0 ? 0 : std::atol(s.substr(0, c).c_str())) * 60 + (c + 1 < s.size() ? std::atol(s.substr(c + 1).c_str()) : 0);
    else {
        const long n = s.empty() ? 0 : std::atol(s.c_str());
        v = s.size() <= 2 ? n : (n / 100) * 60 + n % 100;
    }
    return (int)std::clamp(v, 1L, 600L);
}
inline bool g_dashKeyStep = false;  // the start screen's arrow keys step START TIME; a click types it
inline void dashClick(Game& g, int id, int dir) {
    SimSettings& st = g.set;
    if (g.qCountEdit && id != D_QCOUNT) { if (!g.qCountBuf.empty()) g.qCount = std::clamp(std::atoi(g.qCountBuf.c_str()), 1, 99); g.qCountEdit = false; }
    if (g.timeEdit && id != D_STARTT) {  // clicking elsewhere accepts what was typed
        if (!g.timeBuf.empty()) { st.startSecs = parseStartTime(g.timeBuf); g_matchLen = st.startSecs; g.matchT = g_matchLen; }
        g.timeEdit = false;
    }
    if (id >= D_BOTS && id < D_BOTS + BOT_CHOICE_N) { st.bots = id - D_BOTS; g.botsDirty = true; return; }
    if (id == D_DRIVER) { st.pilot = st.pilot > 0 ? 0 : 1; return; }
    if (id >= D_FOCUS && id < D_FOCUS + 4) {
        int& f = st.focus[id == D_FOCUS ? 3 : id - D_FOCUS - 1];
        f = (f + dir + FOCUS_N) % FOCUS_N;
        return;
    }
    if (id >= D_WHEELBOT && id < D_WHEELBOT + 4) { bool& ov = st.omniBot[id == D_WHEELBOT ? 3 : id - D_WHEELBOT - 1]; ov = !ov; }
    else if (id >= D_LIFTBOT && id < D_LIFTBOT + 4) { int& lv = st.liftBot[id == D_LIFTBOT ? 3 : id - D_LIFTBOT - 1]; lv = (lv + dir + 3) % 3; }
    else if (id >= D_RPMBOT && id < D_RPMBOT + 4) { int& rv = st.rpmBot[id == D_RPMBOT ? 3 : id - D_RPMBOT - 1]; rv = rv == 1 ? 2 : 1; }
    else if (id >= D_SCOREBOT && id < D_SCOREBOT + 4) { int& yv = st.yel[id == D_SCOREBOT ? 3 : id - D_SCOREBOT - 1]; yv = 1 - yv; }
    else if (id == D_LOADS) st.infiniteLoads = !st.infiniteLoads;
    else if (id == D_SCORE) st.hud ^= 1;
    else if (id == D_TIMER) st.hud ^= 2;
    else if (id == D_STARTT) {
        if (!g_dashKeyStep && dir > 0) { g.timeEdit = !g.timeEdit; g.timeBuf.clear(); return; }  // click: type minutes and seconds
        g.timeEdit = false;
        st.startSecs = startStep(st.startSecs, dir); g_matchLen = st.startSecs; g.matchT = g_matchLen;
    }
    else if (id == D_PLAY) g.playRequested = true;
    else if (id == D_SIMONE) {  // one match with these settings, played out at full speed and not shown
        g.bracketCur = -1;
        g.fastSim = true;
        g.playerOff = true;  // a CPU copy of the player's robot plays
        g.botsDirty = true;
        g.playRequested = true;
        g.dash = false;
    }
    else if (id == D_QSIM || id == D_QWATCH) qualStart(g, id == D_QSIM);
    else if (id == D_QALL) { g.simAll = true; g.simLeft = -1; qualStart(g, true); }
    else if (id == D_QFULL) { g.simAll = true; g.autoAll = true; g.simLeft = -1; qualStart(g, true); }
    else if (id == D_QSIMN) { const int n = std::min(g.qCount, qualTotal(g) - g.qDone); if (n > 0) { g.simAll = true; g.simLeft = n; qualStart(g, true); } }
    else if (id == D_QCOUNT) { g.qCountEdit = !g.qCountEdit; g.qCountBuf.clear(); }
    else if (id == D_QALLI) { if (g.stage == 1 && g.qDone >= qualTotal(g)) { formAlliances(g); g.dashPage = 1; g.dashSel = -1; } }
    else if (id == D_QPAGE) { g.dashPage = 6; g.dashSel = -1; }
    else if (id == D_QMPAGE) { const int n = std::max(1, (qualTotal(g) + 11) / 12); const int cur = g.qMPage < 0 ? std::clamp((g.qDone - 1) / 12, 0, n - 1) : g.qMPage; g.qMPage = (cur + (dir > 0 ? 1 : n - 1)) % n; }
    else if (id == D_QSPAGE) g.qSPage ^= 1;
    else if (id == D_LTAB0 || id == D_LTAB1) { g.lbMode = id - D_LTAB0; g.lbPage = 0; }
    else if (id == D_LPAGE) g.lbPage ^= 1;
    else if (id == D_TPAGE) { const int n = QUAL_N / 20; g.teamPage = (g.teamPage + (dir > 0 ? 1 : n - 1)) % n; }
    else if (id >= D_TRPM && id < D_TRPM + QUAL_N) { BTeam& t = g.teams[id - D_TRPM]; t.rpm = t.rpm == 1 ? 2 : 1; }
    else if (id >= D_TLIFT && id < D_TLIFT + QUAL_N) { BTeam& t = g.teams[id - D_TLIFT]; t.lift = (t.lift + dir + 3) % 3; }
    else if (id >= D_TWHEEL && id < D_TWHEEL + QUAL_N) { BTeam& t = g.teams[id - D_TWHEEL]; t.omni = !t.omni; }
    else if (id >= D_BFOCUS && id < D_BFOCUS + 2 * BR_ALLIANCES) {  // a team's strategy in the bracket
        BTeam& t = g.bAl[(id - D_BFOCUS) / 2].t[(id - D_BFOCUS) % 2];
        t.focus = (t.focus + dir + FOCUS_N) % FOCUS_N;
    }
    else if (id >= D_BRPM && id < D_BRPM + 2 * BR_ALLIANCES) {
        BTeam& t = g.bAl[(id - D_BRPM) / 2].t[(id - D_BRPM) % 2];
        t.rpm = t.rpm == 1 ? 2 : 1;
    }
    else if (id >= D_BLIFT && id < D_BLIFT + 2 * BR_ALLIANCES) {
        BTeam& t = g.bAl[(id - D_BLIFT) / 2].t[(id - D_BLIFT) % 2];
        t.lift = (t.lift + dir + 3) % 3;
    }
    else if (id >= D_BINFO && id < D_BINFO + BR_ALLIANCES) { loadAllTime(g); g.infoAl = id - D_BINFO; g.infoFrom = g.dashPage; g.dashPage = 4; g.dashSel = -1; }
    else if (id == D_LEADER) { loadAllTime(g); g.dashPage = 5; g.dashSel = -1; }
    else if (id == D_LBACK) { g.dashPage = g.stage == 2 ? 1 : 6; g.dashSel = -1; }
    else if (id == D_IBACK) { g.dashPage = g.infoFrom; g.dashSel = -1; }
    else if (id >= D_BYEL && id < D_BYEL + 2 * BR_ALLIANCES) {
        BTeam& t = g.bAl[(id - D_BYEL) / 2].t[(id - D_BYEL) % 2];
        t.yel = !t.yel;
    }
    else if (id >= D_BWHEEL && id < D_BWHEEL + 2 * BR_ALLIANCES) {
        BTeam& t = g.bAl[(id - D_BWHEEL) / 2].t[(id - D_BWHEEL) % 2];
        t.omni = !t.omni;
    }
    else if (id == D_BRACKET) {  // a new bracket starts on its teams page; one in progress opens where it is
        if (!g.bracketMade || bracketDone(g)) { bracketNew(g); g.dashPage = 2; }
        else g.dashPage = g.stage == 0 ? 2 : g.stage == 1 ? 6 : 1;
        g.dashSel = -1;
    }
    else if (id == D_BSTART) { if (g.stage == 0) { qualBuild(g); g.dashPage = 6; } else g.dashPage = g.stage == 1 ? 6 : 1; g.dashSel = -1; }
    else if (id == D_BRANDOM) {
        if (g.stage < 2) randomizeTeams(g);
        else for (int i = 0; i < BR_ALLIANCES; ++i) for (auto& t : g.bAl[i].t) { t.rpm = 1 + std::rand() % 2; t.lift = std::rand() % 3; t.omni = false; }
    }  // robot features only: names stay
    else if (id == D_BEDIT) { g.dashPage = 2; g.dashSel = -1; }
    else if (id == D_PSIM) bracketStart(g, true);
    else if (id == D_PWATCH) bracketStart(g, false);
    else if (id == D_PBACK) { g.dashPage = 1; g.dashSel = -1; }
    else if (id == D_BBACK) { g.dashPage = 0; g.dashSel = -1; }
    else if (id == D_BNEW) { bracketNew(g); g.dashPage = 2; g.dashSel = -1; }
    else if ((id == D_BSIM || id == D_BWATCH) && bracketNext(g) >= 0) {  // first the match preview: strategies
        g.previewFast = id == D_BSIM;
        g.dashPage = 3;
        g.dashSel = -1;
    }
    else if (id == D_BSIMALL && bracketNext(g) >= 0) { g.simAll = true; bracketStart(g, true); }
}

// Diagram pixel -> sim pixel. The diagram's playing surface spans x 112..489, y 96..473.
inline double gx(double ix) { return (ix - 112.0) / 377.0 * 720.0; }
inline double gy(double iy) { return (iy - 96.0) / 377.0 * 720.0; }

// Quadrants are the four triangles cut by the field diagonals.
inline int quadrantOf(double x, double y) {
    double dx = x - 360.0, dy = y - 360.0;
    if (std::fabs(dx) >= std::fabs(dy)) return dx < 0 ? 0 : 1;
    return dy > 0 ? 2 : 3;
}

inline double goalTopIn(const Game& g, int i) { return GOAL_H_IN[i] + HALF_IN * g.stack[i].size(); }

// Height of the bottom of a loose piece: a standing pin in a cup on the floor sits in the cup's
// top half (3.25 in up); everything else rests on the floor.
inline double floorBottomIn(const Game& g, int idx) {
    const Piece& pc = g.p[idx];
    if (!pc.isPin || pc.lying || !loose(pc)) return 0.0;
    for (const Piece& o : g.p)
        if (!o.isPin && !o.lying && loose(o) && std::hypot(o.x - pc.x, o.y - pc.y) < 0.6 * PX_IN) return HALF_IN;
    return 0.0;
}
inline double stackBottomIn(int goal, int k) { return GOAL_H_IN[goal] - HALF_IN + HALF_IN * k; }
// Bottom of the k-th piece in a Loader (they rest on the floor inside it; a pin and a cup nest
// half a piece deep, two of the same kind sit end to end)
// Where a Loader's bottom piece waits: in the opening at the foot of the Loader, on the field side
inline double loaderOutX(int li) { return LOADERS[li].x + (LOADERS[li].x < FIELD_IN / 2 ? 2.2 : -2.2); }
constexpr int LOADER_CAPACITY = 2;  // a pin and a cup stacked (see loaderHasRoom)
constexpr double LOADER_GATE_UP_S   = 0.25;  // the clear cover lifts this fast when the robot comes up to the Loader
constexpr double LOADER_GATE_DOWN_S = 0.40;  // and drops back more slowly once it leaves
constexpr double LOADER_APPROACH_IN = 11.0;  // the player's robot within this of the foot opening lifts the clear tube
constexpr double LOADER_SLIDE_S     = 0.22;  // the bottom piece slides out to the opening once the cover is up
constexpr double LOADER_BOUNCE_E    = 0.28;  // a piece landing on the stack bounces with this restitution
// Resting bottom height of the k-th piece in a Loader. They stack in the tube, each resting on the
// one below it: a pin and a cup nest half a piece deep (3.25 in), two of the same kind sit end to end
// (6.5 in). While the bottom piece is sliding out to the foot opening, the next one rests above it
// at a full piece height (it can't nest into a piece that is leaving), until the bottom one is gone.
inline double loaderRestIn(const Game& g, int li, int k) {
    const auto& st = g.loaderStack[li];
    double z = 0.0;
    for (int j = 1; j <= k && j < (int)st.size(); ++j) {
        double step = (g.p[st[j]].isPin != g.p[st[j - 1]].isPin) ? HALF_IN : PIECE_H_IN;
        if (j == 1 && g.p[st[0]].slide > 0.05 && g.p[st[1]].isPin == g.p[st[0]].isPin) step = PIECE_H_IN;  // a nested pin / cup slides out with it
        z += step;
    }
    return z;
}
// Where a piece of this kind dropped in now would come to rest.
inline double loaderNextRestIn(const Game& g, int li, bool isPin) {
    const auto& st = g.loaderStack[li];
    if (st.empty()) return 0.0;
    double step = (g.p[st.back()].isPin != isPin) ? HALF_IN : PIECE_H_IN;
    if (st.size() == 1 && g.p[st[0]].slide > 0.05) step = PIECE_H_IN;
    return loaderRestIn(g, li, (int)st.size() - 1) + step;
}
// A Loader holds a pin alone, a cup alone, or a pin and a cup stacked together (either order):
// never two of the same kind, and nothing more once it holds both (the V5RC Override Playground's
// Match Load rule).
inline bool loaderHasRoom(const Game& g, int li, bool isPin) {
    const auto& st = g.loaderStack[li];
    return st.empty() || (st.size() == 1 && g.p[st[0]].isPin != isPin);
}
inline double loaderBottomIn(const Game& g, int li, int k) {
    const auto& st = g.loaderStack[li];
    double z = 0.0;
    for (int j = 1; j <= k && j < (int)st.size(); ++j)
        z += (g.p[st[j]].isPin != g.p[st[j - 1]].isPin) ? HALF_IN : PIECE_H_IN;
    return z;
}

inline void fieldSetup() {
    Game& g = game();
    g.p.clear();
    for (auto& s : g.stack) s.clear();
    for (auto& s : g.loaderStack) s.clear();
    g.toggleTurn = {};
    g.held = -1;
    g.rider = -1;
    g.ratchetArmed = false;
    g.overGoal = -1;
    g.toggle = {{C_YEL, C_YEL, C_YEL, C_YEL}};
    g.touching = {};

    // Positions in field inches from the bottom-left corner (x right, y up), read off the manual's
    // scoring-object diagram and snapped to the official grid (23.11 / 46.66 / 70.20 / 93.75 / 117.30, Appendix A).
    // Counts match the manual: 36 cups (24 gray side up, 12 clear side up), 17 yellow, 8 red/yellow,
    // 8 blue/yellow and 4 red/blue pins on the field, plus the preload.
    auto X = [](double xin) { return xin * PX_IN; };
    auto Y = [](double yin) { return 720.0 - yin * PX_IN; };
    auto cup = [&](double xin, double yin, bool grayUp) {
        Piece c{false, X(xin), Y(yin), C_YEL, C_YEL};
        c.grayUp = grayUp;
        g.p.push_back(c);
    };
    auto pin = [&](double xin, double yin, Col t, Col b) { g.p.push_back({true, X(xin), Y(yin), t, b}); };
    auto lyingPin = [&](double xin, double yin, double dxIn, double dyIn, Col tip) {
        Piece pc{true, X(xin + dxIn * 4.8), Y(yin + dyIn * 4.8), tip, C_YEL};
        pc.lying = true;
        pc.ang = std::atan2(-dyIn, dxIn);  // screen y points down
        g.p.push_back(pc);
    };

    // Walls: on each wall, two groups of gray-up cup, gray-up cup holding a yellow pin, gray-up cup
    const double W = 1.58, F = 140.41 - 1.58, SP = 3.16;  // wall groups touch (official layout)
    for (double a : {46.66, 93.75}) {
        for (auto wall : {0, 1, 2, 3}) {
            for (int k = -1; k <= 1; ++k) {
                double along = a + k * SP;
                double x = wall == 0 ? W : wall == 1 ? F : along;
                double y = wall == 2 ? W : wall == 3 ? F : along;
                cup(x, y, true);
                if (k == 0) pin(x, y, C_YEL, C_YEL);
            }
        }
    }
    // Center diamond: red/blue pins in clear-up cups, alternating which color is up
    cup(70.20, 93.75, false); pin(70.20, 93.75, C_BLUE, C_RED);
    cup(93.75, 70.20, false); pin(93.75, 70.20, C_BLUE, C_RED);
    cup(46.66, 70.20, false); pin(46.66, 70.20, C_RED, C_BLUE);
    cup(70.20, 46.66, false); pin(70.20, 46.66, C_RED, C_BLUE);
    // Bottom-left and top-right diagonals: yellow pins in clear-up cups
    for (auto c : {std::pair<double, double>{23.11, 23.11}, {46.66, 46.66}, {93.75, 93.75}, {117.30, 117.30}}) {
        cup(c.first, c.second, false);
        pin(c.first, c.second, C_YEL, C_YEL);
    }
    // Top-left and bottom-right diagonals: a clear-up cup with four pins lying around it, yellow
    // ends inward (up and right: blue tips; left and down: red tips)
    for (auto c : {std::pair<double, double>{23.11, 117.30}, {46.66, 93.75}, {93.75, 46.66}, {117.30, 23.11}}) {
        cup(c.first, c.second, false);
        lyingPin(c.first, c.second, 0, 1, C_BLUE);
        lyingPin(c.first, c.second, 1, 0, C_BLUE);
        lyingPin(c.first, c.second, -1, 0, C_RED);
        lyingPin(c.first, c.second, 0, -1, C_RED);
    }
    // The five black goals start with a yellow/yellow pin in them
    for (int i = 4; i < 9; ++i) {
        Piece pc{true, GOAL_POS_IN[i][0] * PX_IN, 720.0 - GOAL_POS_IN[i][1] * PX_IN, C_YEL, C_YEL};
        pc.goal = i;
        g.stack[i].push_back((int)g.p.size());
        g.p.push_back(pc);
    }
    // The red robot starts holding its preload, red half up
    Piece pre{true, 0, 0, C_RED, C_YEL};
    pre.held = true;
    g.p.push_back(pre);
    g.held = (int)g.p.size() - 1;
    g.heldOffX = g.heldOffY = 0.0;
}

inline void stopAllMotors(SimState& s) {
    for (int i = 1; i < MAX_PORTS; ++i) {
        s.motors[i].voltage.store(0);
        s.motors[i].target_velocity.store(0);
        s.motors[i].millivolts.store(0);
    }
}

// Put the field, robot and mechanisms back to the start: lift down, claw gripping the preload.
inline void resetAll(SimState& s) {
    Game& g = game();
    {
        std::lock_guard<std::mutex> lock(s.pose_mutex);
        double sx = g.sx, sy = g.sy, sh = g.sh;
        s.pose.heading_accumulated += sh - s.pose.heading;
        s.pose.x = sx;
        s.pose.y = sy;
        s.pose.heading = sh;
    }
    for (Bot& b : g.bots) b.carry = b.rider = b.target = b.goal = -1;  // the pieces are about to be made again
    g.pilot.on = false;  // the CPU driving the player's robot starts afresh
    stopAllMotors(s);
    s.motors[LIFT_PORT].set_position_deg(0.0);
    s.motors[LIFT_PORT2].set_position_deg(0.0);
    s.motors[CLAW_PORT].set_position_deg(gripDeg(PIN_W_IN));
    fieldSetup();
    g.botsDirty = true;
    g_matchLen = g.set.startSecs;
    g.matchT = g_matchLen;
    g.matchOver = false;
    g.loadT = {};
    g.minors = {}; g.dq = {}; g.playerZoneT = 0.0; g.violMsg.clear(); g.violMsgT = 0.0;
    g.playerGoalT = g.playerLiftT = g.playerHitCool = 0.0; g.loadYellow = {{1, 1}};
    g.loaderGate = {};  // the Loaders start down
    g.loadPins = {{MATCH_LOAD_PINS, MATCH_LOAD_PINS}};
    g.loadCups = {{MATCH_LOAD_CUPS, MATCH_LOAD_CUPS}};
    g.prevGapIn = g.gapIn = PIN_W_IN;
    g.pitch = g.pitchV = g.roll = g.rollV = 0.0;  // back on its wheels
    g.tipped = false;
    g.prevRValid = false;
}

// ── Centre of mass and tipping ────────────────────────────────────────────────
// Where the robot rests on the floor: the front and back omni wheels' contact lines, and the
// outer edges of the wheel treads (13 in track, 1.1 in wide tires). Robot-frame inches.
constexpr double TIP_EDGE_X_IN   = 6.1;   // front and back wheel contact lines (WHEEL_X_IN[0] and -WHEEL_X_IN[2])
constexpr double TIP_EDGE_Y_IN   = 6.8;
constexpr double G_IN_S2         = 386.1;
constexpr double TIP_LAND_RAD    = 1.30;  // a fallen robot comes to rest on its bumper and arms, about 75 deg over
constexpr double IMPACT_TIP_SHARE = 0.4;  // share of a crash's rigid-body kick that reaches the tipping (flex, rubber, wheel slip take the rest)
constexpr double TIP_CORNER_Y_IN  = 7.5;  // outer drive rails, which touch down when it rolls far onto its side
constexpr double TIP_CORNER_Z_IN  = 1.4;  // bottom of the drive rails
constexpr double FRAME_HIT_KEEP   = 0.5;  // lean speed kept when the frame lands on the floor (the rest goes into the hit)
constexpr double ROBOT_CATCH_RAD  = 0.12; // leaning toward something it is against, it rests on it about 7 deg over
// Pieces as rigid bodies (pieceDynamics part of gameUpdate)
constexpr double PIN_TIP_R_IN    = 0.70;  // a standing pin balances on its 1.40 in tip
constexpr double CUP_RIM_R_IN    = 1.58;  // a cup stands on its 3.16 in rim
constexpr double FLOOR_MU        = 0.45;  // plastic sliding on the foam tiles
constexpr double ROBOT_HIT_Z_IN  = 2.2;   // height the chassis rails hit a piece at
constexpr double PIN_MASS_LB     = 0.06;  // very light hollow plastic pieces (next to an 11.7 lb robot, almost nothing)
constexpr double CUP_MASS_LB     = 0.05;
constexpr double PIECE_BOUNCE_E  = 0.4;   // restitution, plastic on plastic
constexpr double PIN_ROLL_MU     = 0.10;  // rolling resistance of a lying pin: its collar is an octagon, so it rolls a bit bumpily
constexpr double CUP_ROLL_MU     = 0.04;  // a lying cup rolls on its two round rims
// A piece leaning over its base edge is down when its widest part (the 1.58 in collar or rim, half
// its length up) reaches the floor: atan(3.25 / (1.58 - base radius)). A pin (0.70 base) lands at
// about 75 deg, a cup (on its rim) at exactly 90, on both rims.
inline double pieceLandRad(double baseR) { return std::atan2(HALF_IN, std::max(0.0, PIECE_MAX_R_IN - baseR)); }

// Mass and centre of mass, worked out from the robot's parts every step, so it moves as the
// DR6B rises and folds and when a piece is held. Masses are estimates for V5 parts (pounds):
// 11 W motor 0.34, 5.5 W motor 0.21, battery 0.84, brain 0.55, 3.25 in omni 0.25.
inline void robotMassProps(Game& g, double hl) {
    struct M { double m, x, y, z; };
    std::vector<M> parts;
    parts.push_back({2.40, 0.0, 0.0, 2.1});              // C-channel rails, cross members, plates, screws
    parts.push_back({1.50, 0.0, 0.0, 1.625});            // four omni wheels, two traction wheels and their 48-tooth gears
    parts.push_back({4 * 0.34 + 2 * 0.21, (WHEEL_X_IN[0] + WHEEL_X_IN[1] + WHEEL_X_IN[2]) / 3 + 1.48, 0.0, 3.1});  // drive motors, up and ahead of their wheels
    parts.push_back({0.84, -hl + 3.2, 0.0, 2.4});        // battery
    parts.push_back({0.55, -hl + 3.2, 0.0, 4.0});        // brain
    const double xt = hl - DR4B_TOWER_BACK_IN, z0 = dr4bTowerPivotIn();
    parts.push_back({0.60, xt, 0.0, (2.6 + ROBOT_START_H_IN) / 2});  // the two main posts
    parts.push_back({2 * 0.34 + 0.20, xt, 0.0, z0 - 2.05});          // lift motors and 84-tooth gears
    {
        double padX, padZ;
        ramPadPose(g.liftAngle, hl, padX, padZ);
        parts.push_back({0.25, padX + 2.0, 0.0, padZ});  // toggle arms on the back of the lift
    }
    parts.push_back({0.12, hl - 4.0, 0.0, 0.55});        // goal aligner
    // DR6B sections (bars both sides, joints), same geometry as the 3D drawing
    const double L = DR4B_BAR_IN, sp = DR4B_SPACING_IN, cth = std::cos(g.liftAngle), sth = std::sin(g.liftAngle);
    double xs = xt, zb = z0, xe = xt;
    const bool barLift = g_liftType == 1 || g_liftType == 2;
    if (!barLift) parts.push_back({0.5 * g_liftSections, xt, 0.0, g.liftIn + 2.0});  // cascade slides or clawbot arm
    for (int k = 0; k < g_liftSections && barLift; ++k) {
        const int half = g_liftSections / 2;
        const double Lk = secLen(k);
        if (k > 0) { xs = xe + (k == half ? 2 * DR4B_MID_GEAR_R : 0.0); zb += secLen(k - 1) * sth + sp; }
        xe = xs + (k < half ? -Lk * cth : Lk * cth);
        parts.push_back({0.22 * Lk / DR4B_BAR_IN, (xs + xe) / 2, 0.0, zb + Lk * sth / 2 + sp / 2});
        if (k + 1 == half) parts.push_back({0.25, xe, 0.0, zb + Lk * sth + sp});  // middle tower plates and gear pair
    }
    const double clawX = hl + REACH_IN;
    parts.push_back({1.30, clawX, 0.0, g.liftIn + 3.0});  // carriage, claw plate, wheel stacks, claw motor
    auto pieceLb = [&](int i) { return g.p[i].isPin ? PIN_MASS_LB : CUP_MASS_LB; };  // very light hollow plastic pieces
    if (g.held >= 0) parts.push_back({pieceLb(g.held), clawX, 0.0, g.liftIn + HALF_IN});
    if (g.rider >= 0) parts.push_back({pieceLb(g.rider), clawX, 0.0, g.liftIn + 2 * HALF_IN});

    double m = 0, sx = 0, sy = 0, sz = 0;
    for (const M& p : parts) { m += p.m; sx += p.m * p.x; sy += p.m * p.y; sz += p.m * p.z; }
    g.massLb = m;
    g.comX = sx / m; g.comY = sy / m; g.comZ = sz / m;
    // Radius of gyration squared about the centre of mass (point masses plus the spread of each part)
    double ip = 0, ir = 0;
    for (const M& p : parts) {
        double dx = p.x - g.comX, dy = p.y - g.comY, dz = p.z - g.comZ;
        ip += p.m * (dx * dx + dz * dz);
        ir += p.m * (dy * dy + dz * dz);
    }
    g.k2Pitch = ip / m + 12.0;
    g.k2Roll = ir / m + 10.0;
    // Steady acceleration that would just lift the wheels on one side: edge distance / COM height
    const double xF = TIP_EDGE_X_IN;
    (void)hl;
    g.tipBrakeG = (xF - g.comX) / g.comZ;
    g.tipAccelG = (xF + g.comX) / g.comZ;
    g.tipSideG = (TIP_EDGE_Y_IN - std::fabs(g.comY)) / g.comZ;
}

// One side the robot can lean over. It first rocks on its wheels (the edge d in from the centre of
// mass, on the floor). Once the lean reaches atan(cz / (cd - d)) the frame touches down (a corner cd
// in from the COM and cz above the floor: the back rails, the aligner at the front, the outer
// rails at the sides) and from then on it pivots on that corner, which is much harder to roll over.
// lim: something it is against catches it at that lean.
struct TipSide { double d, cd, cz, lim; };
inline double tipFrameRad(const TipSide& S) { return std::atan2(S.cz, std::max(0.01, S.cd - S.d)); }

// ang >= 0 leans over side P, ang <= 0 over side N. push: the pseudo-acceleration toward P that the
// robot feels (braking pushes the body forward over the front). Gravity pulls it back while the COM
// is inside the pivot and over once the COM passes it.
inline void tipAxis(double& ang, double& vel, const TipSide& P, const TipSide& N, double h, double k2,
                    double push, double dt, bool& fell) {
    const double fP = tipFrameRad(P), fN = tipFrameRad(N);
    // Pivot at lean a: horizontal COM distance in from it and COM height above it
    auto pivot = [&](const TipSide& S, double f, double a, double& d, double& hh) {
        if (a < f) { d = S.d; hh = h; } else { d = S.cd; hh = h - S.cz; }
    };
    auto torque = [&](const TipSide& S, double f, double a, double sPush) {  // per unit mass
        double d, hh;
        pivot(S, f, a, d, hh);
        double X = d * std::cos(a) - hh * std::sin(a), H = d * std::sin(a) + hh * std::cos(a);
        return -G_IN_S2 * X + sPush * H;
    };
    auto inertia = [&](const TipSide& S, double f, double a) {
        double d, hh;
        pivot(S, f, a, d, hh);
        return k2 + d * d + hh * hh;
    };
    const int n = std::max(1, (int)std::ceil(dt / 0.001));
    const double hs = dt / n;
    for (int i = 0; i < n; ++i) {
        if (ang == 0.0 && vel == 0.0) {
            // On all its wheels: it only starts to lean if the push beats gravity about an edge
            if (torque(P, fP, 0, push) > 0)       vel = 1e-6;
            else if (torque(N, fN, 0, -push) > 0) vel = -1e-6;
            else continue;
        }
        if (ang > 0 || (ang == 0 && vel > 0)) {
            const double before = ang;
            vel += torque(P, fP, ang, push) / inertia(P, fP, ang) * hs;
            ang += vel * hs;
            if (before < fP && ang >= fP && vel > 0) vel *= FRAME_HIT_KEEP;  // the frame lands on the floor
            if (ang <= 0) { ang = 0; vel = vel < -0.6 ? -0.2 * vel : 0.0; }  // drops back onto the wheels, small bounce
        } else {
            const double before = -ang;
            vel -= torque(N, fN, -ang, -push) / inertia(N, fN, -ang) * hs;
            ang += vel * hs;
            if (before < fN && -ang >= fN && vel < 0) vel *= FRAME_HIT_KEEP;
            if (ang >= 0) { ang = 0; vel = vel > 0.6 ? -0.2 * vel : 0.0; }
        }
        // Something on that side (a wall, goal, Loader or Toggle the robot is against) catches it:
        // it leans onto it and rocks back instead of going over
        const double lim = ang > 0 ? P.lim : N.lim;
        if (std::fabs(ang) >= lim) {
            if (lim < TIP_LAND_RAD) {
                ang = ang > 0 ? lim : -lim;
                if (ang * vel > 0) vel = -0.25 * vel;
                continue;
            }
            ang = ang > 0 ? TIP_LAND_RAD : -TIP_LAND_RAD;  // over and down on its side
            vel = 0.0;
            fell = true;
            return;
        }
    }
}

// aF / aL: the chassis' acceleration from the wheels this step (forward / left, in/s^2).
// dvF / dvL: sudden speed change from hitting something (in/s), at height zc (in).
// brace[0..3]: front, back, left, right are against something (it can only lean that way a little)
inline void robotTipStep(Game& g, double hl, double aF, double aL, double dvF, double dvL, double zc, double dt,
                         const bool brace[4] = nullptr) {
    g.accF = aF; g.accL = aL;
    if (!g.set.tipping) { g.pitch = g.pitchV = g.roll = g.rollV = 0.0; g.tipped = false; return; }
    if (g.tipped) return;  // lying on its side until a reset
    if (dt <= 0) return;
    const double xF = TIP_EDGE_X_IN, h = g.comZ;
    bool fell = false;
    // The wheels push at the floor and a collision pushes at height zc. Both go into one pseudo-push
    // (per unit mass, as if acting at the COM height) so they balance properly: driving into a wall
    // leaves only the small couple between the floor and the bumper, while a fast hit below the COM
    // throws the top over.
    const double kHit = (h - zc) / (h * dt);
    // Split each collision into the steady part (the wall holding back what the wheels just pushed,
    // which must cancel the wheel push exactly) and the crash beyond it. A real robot isn't rigid:
    // the aluminium flexes, the rubber pads squash and the wheels slip, so only part of a crash's
    // rigid-body kick goes into tipping it (IMPACT_TIP_SHARE).
    auto effDv = [&](double a, double dv) {
        double steady = -a * dt;
        if (steady * dv <= 0) steady = 0;                        // the wheels weren't pushing into it
        else if (std::fabs(steady) > std::fabs(dv)) steady = dv;
        return steady + IMPACT_TIP_SHARE * (dv - steady);
    };
    const double pushF = -aF - effDv(aF, dvF) * kHit, pushL = -aL - effDv(aL, dvL) * kHit;
    const double catchR = ROBOT_CATCH_RAD;
    const bool bF = brace && brace[0], bB = brace && brace[1], bL = brace && brace[2], bR = brace && brace[3];
    // Pitch: + leans over the front wheels (braking or a front hit push it that way), then onto the
    // aligner's lip; - over the back wheels, then onto the back rails
    const TipSide front{xF - g.comX, hl + ALIGN_MOUTH_X - g.comX, ALIGN_Z0, bF ? catchR : TIP_LAND_RAD};
    const TipSide back{xF + g.comX, hl + g.comX, TIP_CORNER_Z_IN, bB ? catchR : TIP_LAND_RAD};
    tipAxis(g.pitch, g.pitchV, front, back, h, g.k2Pitch, pushF, dt, fell);
    (void)zc;
    // Roll: + leans over the left wheels (accelerating to the right pushes it left), then the outer rail
    const TipSide left{TIP_EDGE_Y_IN - g.comY, TIP_CORNER_Y_IN - g.comY, TIP_CORNER_Z_IN, bL ? catchR : TIP_LAND_RAD};
    const TipSide right{TIP_EDGE_Y_IN + g.comY, TIP_CORNER_Y_IN + g.comY, TIP_CORNER_Z_IN, bR ? catchR : TIP_LAND_RAD};
    tipAxis(g.roll, g.rollV, left, right, h, g.k2Roll, pushL, dt, fell);
    if (fell) g.tipped = true;
}

inline void scoreField(Game& g, int& red, int& blue, std::array<int, 4>* by) {
    red = blue = 0;
    for (int i = 0; i < 9; ++i) {
        const auto& st = g.stack[i];
        // The Midfield Goal's yellow Pins belong to the alliance with more robots in the Midfield at the
        // end (none on a tie, <SC5b>); a Quadrant's to the colour its Toggle is set to (<SC5a>)
        const int q = quadrantOf(GOAL_POS_IN[i][0] * PX_IN, 720.0 - GOAL_POS_IN[i][1] * PX_IN);
        bool touched = g.touching[q];  // <SC4b>: a Toggle in contact with a robot is not set to a colour
        for (const Bot& bt : g.bots) touched = touched || (bt.on && bt.touchTog[q]);
        Col owner = (i == 8) ? (g.midRed > g.midBlue ? C_RED : g.midBlue > g.midRed ? C_BLUE : C_YEL)
                             : (touched ? C_YEL : g.toggle[q]);
        for (size_t k = 0; k < st.size(); ++k) {
            const Piece& pc = g.p[st[k]];
            if (!pc.isPin) continue;
            bool botVisible = (k > 0) && !g.p[st[k - 1]].grayUp;               // in a cup's clear top half
            bool topVisible = !(k + 1 < st.size() && !g.p[st[k + 1]].grayUp);  // not under a cup's opaque bottom
            for (int h = 0; h < 2; ++h) {
                if (h == 0 ? !topVisible : !botVisible) continue;
                Col c = h == 0 ? pc.top : pc.bot;
                int pts = 0;
                if (c == C_RED) { red += 5; pts = 5; }
                else if (c == C_BLUE) { blue += 5; pts = 5; }
                else if (owner == C_RED) { red += 10; pts = 10; }
                else if (owner == C_BLUE) { blue += 10; pts = 10; }
                if (by && pts && pc.placedBy >= 0 && pc.placedBy < 4) (*by)[pc.placedBy] += pts;
            }
        }
    }
}

// Drop what the claw holds where it is (the gripped piece, plus the piece nested on it, if any):
// into a Loader if the claw is over its tube and high enough, onto the goal stack under it if
// the gripped piece fits, else onto the floor.
inline void releaseHeld(Game& g, const std::array<GoalObstacle, 9>& goals, double cx, double cy, double c, double sn) {
    int idx = g.held, rid = g.rider;
    Piece& pc = g.p[idx];
    pc.held = false;
    // Both fall from where the claw let go of them (the drop is drawn under gravity)
    pc.fz = g.liftIn; pc.fzv = 0.0;
    if (rid >= 0) { g.p[rid].fz = g.liftIn; g.p[rid].fzv = 0.0; }
    g.held = -1;
    g.rider = -1;
    g.ratchetArmed = false;  // re-arms once the jaws are clear of pieces
    if (rid >= 0) g.p[rid].held = false;
    for (int li = 0; li < 4; ++li) {
        double lx = LOADERS[li].x * PX_IN, ly = 720.0 - LOADERS[li].y * PX_IN;
        if (g.liftIn >= LOADER_H - 0.3 && std::hypot(cx - lx, cy - ly) <= (LOADER_TUBE_D / 2 + 0.8) * PX_IN &&
            (rid >= 0 ? g.loaderStack[li].empty() : loaderHasRoom(g, li, pc.isPin))) {  // a pin and cup together need an empty one; else it is rejected
            for (int k : {idx, rid}) {
                if (k < 0) continue;
                g.p[k].loader = li;
                g.p[k].lying = false;
                g.p[k].lz = LOADER_H;
                g.p[k].lzv = 0.0;
                g.p[k].slide = 0.0;
                g.p[k].fz = 0.0;
                g.p[k].x = lx;
                g.p[k].y = ly;
                g.loaderStack[li].push_back(k);
            }
            g.overGoal = -1;
            return;
        }
    }
    int i = g.overGoal;
    if (i >= 0 && g.overGoalOk && std::hypot(cx - goals[i].x, cy - goals[i].y) <= 13.0) {
        auto& st = g.stack[i];
        bool fits = st.empty() ? pc.isPin : (g.p[st.back()].isPin != pc.isPin);
        if (fits) {
            for (int k : {idx, rid}) {
                if (k < 0) continue;
                g.p[k].fz = std::max(0.0, g.liftIn - stackBottomIn(i, (int)st.size()));
                st.push_back(k);
                g.p[k].goal = i;
                g.p[k].x = goals[i].x;
                g.p[k].y = goals[i].y;
            }
            g.overGoal = -1;
            return;
        }
    }
    // To the floor, clear of the goals
    for (int k = 0; k < 9; ++k) {
        double d = std::hypot(pc.x - goals[k].x, pc.y - goals[k].y);
        double minD = goals[k].r + PIECE_MAX_R_IN * PX_IN;
        if (d < minD) {
            double ux = d > 1e-6 ? (pc.x - goals[k].x) / d : -c;
            double uy = d > 1e-6 ? (pc.y - goals[k].y) / d : -sn;
            pc.x = goals[k].x + ux * minD;
            pc.y = goals[k].y + uy * minD;
        }
    }
    for (const LoaderDef& ld : LOADERS) {  // not inside a Loader's base either
        double lx = ld.x * PX_IN, ly = 720.0 - ld.y * PX_IN;
        double d = std::hypot(pc.x - lx, pc.y - ly), minD = (2.1 + PIECE_MAX_R_IN) * PX_IN;
        if (d < minD) {
            double ux = d > 1e-6 ? (pc.x - lx) / d : (ld.x < FIELD_IN / 2 ? 1.0 : -1.0), uy = d > 1e-6 ? (pc.y - ly) / d : 0.0;
            pc.x = lx + ux * minD;
            pc.y = ly + uy * minD;
        }
    }
    if (rid >= 0) {
        Piece& r = g.p[rid];
        if (r.isPin) {  // a pin in a cup stays standing in it
            r.x = pc.x;
            r.y = pc.y;
        } else {        // a cup on a pin slides off beside it
            r.x = pc.x - c * 3.4 * PX_IN;
            r.y = pc.y - sn * 3.4 * PX_IN;
        }
    }
    g.overGoal = -1;
}

// ── Other robots ─────────────────────────────────────────────────────────────
// Separating-axis test for two rectangles (centre, heading, half length along the heading, half
// width). If they overlap, (mx, my) is the shortest move that takes A out of B.
inline bool obbPush(double ax, double ay, double ah, double ahl, double ahw,
                    double bx, double by, double bh, double bhl, double bhw, double& mx, double& my) {
    const double axes[4][2] = {{std::cos(ah), std::sin(ah)}, {-std::sin(ah), std::cos(ah)},
                               {std::cos(bh), std::sin(bh)}, {-std::sin(bh), std::cos(bh)}};
    auto rad = [](double h, double hl, double hw, const double* u) {
        return hl * std::fabs(std::cos(h) * u[0] + std::sin(h) * u[1]) + hw * std::fabs(-std::sin(h) * u[0] + std::cos(h) * u[1]);
    };
    const double dx = bx - ax, dy = by - ay;
    double best = 1e18;
    mx = my = 0.0;
    for (const auto& u : axes) {
        double d = dx * u[0] + dy * u[1];
        double o = rad(ah, ahl, ahw, u) + rad(bh, bhl, bhw, u) - std::fabs(d);
        if (o <= 0) return false;
        if (o < best) {
            best = o;
            double sg = d > 0 ? -1.0 : 1.0;
            mx = u[0] * o * sg;
            my = u[1] * o * sg;
        }
    }
    return true;
}

inline double wrapPi(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

// Drop what a robot carries onto the floor in front of it, and forget its plan
inline void botRelease(Game& g, Bot& b) {
    if (b.carry >= 0 && b.carry < (int)g.p.size()) {
        Piece& pc = g.p[b.carry];
        pc.carrier = -1;
        pc.fz = 3.0;
        pc.fzv = pc.vx = pc.vy = 0.0;
    }
    if (b.rider >= 0 && b.rider < (int)g.p.size()) {
        Piece& rp = g.p[b.rider];
        rp.carrier = -1;
        rp.fz = 3.0 + HALF_IN;
        rp.fzv = rp.vx = rp.vy = 0.0;
    }
    b.carry = b.rider = b.target = b.goal = -1;
}

// Put the robots the OTHER ROBOTS setting asks for at their starting spots, clear of the goals,
// the wall pieces and the player's start
inline void botsSetup(Game& g) {
    g.togOwner = {{-1, -1, -1, -1}};
    static const char* NUMS[BOT_N] = {"41658F", "41658C", "41658D", "41658F"};
    // Blue starts where red does, turned half way round the field centre (fair matches): partner
    // (13, 28), the player (13, 70.2); so blue (127.4, 112.4) and (127.4, 70.2)
    // Legal starts (<SG1>): one robot per Quadrant, touching its own alliance's wall, clear of Goals,
    // Loaders, Load Zones, Toggles and the wall pieces. Red: the player's robot in the left Quadrant
    // against the left wall between the upper wall pieces and the top Load Zone (9, 107.75) facing the
    // field; the partner in the bottom Quadrant against the bottom wall (27, 9) facing up the field.
    // Blue: the same spots turned half way round the field.
    const double START[BOT_N][3] = {{27.0, 9.0, -M_PI / 2}, {FIELD_IN - 9.0, FIELD_IN - 107.75, M_PI},
                                    {FIELD_IN - 27.0, FIELD_IN - 9.0, M_PI / 2}, {9.0, 107.75, 0.0}};  // field in, sim heading
    static bool seeded = false;
    if (!seeded) { seeded = true; std::srand((unsigned)std::time(nullptr)); }
    for (int i = 0; i < BOT_N; ++i) {
        botRelease(g, g.bots[i]);
        Bot b;
        b.on = i < 3 ? ((BOT_CHOICES[std::clamp(g.set.bots, 0, BOT_CHOICE_N - 1)] >> i) & 1) != 0 : g.playerOff;
        b.team = (i == 0 || i == 3) ? C_RED : C_BLUE;
        b.num = NUMS[i];
        b.level = i < 3 ? g.set.level[i] : g.set.pilotLevel;
        b.focusSet = i < 3 ? g.set.focus[i] : g.set.focus[3];
        b.scoreYel = (i < 3 ? g.set.yel[i] : g.set.yel[3]) != 0;
        { static const double RP[4] = {200.0, 360.0, 450.0, 600.0}; b.topIps = M_PI * 3.25 * RP[std::clamp(g.set.rpmBot[i < 3 ? i : 3], 0, 3)] / 60.0; }
        b.lift = std::clamp(g.set.liftBot[i < 3 ? i : 3], 0, 3);
        if (g.bracketCur >= 0) {  // a tournament match: these alliances' teams
            const BMatch& m = g.bM[g.bracketCur];
            const BAlliance& A = g.bAl[m.a];
            const BAlliance& B = g.bAl[m.b];
            const BTeam& t = i == 0 ? A.t[1] : i == 1 ? B.t[0] : i == 2 ? B.t[1] : A.t[0];
            b.on = i < 3 || g.playerOff;
            b.num = t.num.c_str();
            b.level = t.level;
            b.focusSet = t.focus;
            static const double RPT[4] = {200.0, 360.0, 450.0, 600.0};
            b.topIps = M_PI * 3.25 * RPT[std::clamp(t.rpm, 0, 3)] / 60.0;
            b.lift = std::clamp(t.lift, 0, 3);
            b.omni = t.omni;
            b.scoreYel = t.yel;
        }
        else b.omni = g.set.omniBot[i < 3 ? i : 3];
        b.x = b.chkX = START[i][0] * PX_IN;
        b.y = b.chkY = 720.0 - START[i][1] * PX_IN;
        b.h = START[i][2];
        b.wait = 0.4 + 0.4 * i;
        g.bots[i] = b;
    }
    // Each robot starts holding its Preload: a red/yellow Pin for red, blue/yellow for blue (<SG5>)
    if (g.matchT >= g_matchLen - 0.5) {
        for (int i = 0; i < BOT_N; ++i) {
            Bot& b = g.bots[i];
            if (!b.on) continue;
            Piece pre{true, b.x, b.y, b.team, C_YEL};
            pre.carrier = i;
            b.carry = (int)g.p.size();
            g.p.push_back(pre);
        }
    }
    // The player's robot sits out: its Preload goes away with it
    if (g.playerOff && g.held >= 0) {
        Piece& hp = g.p[g.held];
        hp.held = false;
        hp.carrier = 90;
        hp.x = hp.y = -5000.0;
        g.held = -1;
    }
}

// Can a piece of this kind go on top of this goal's stack? (a pin first, then cup and pin in turn)
inline bool botFits(const Game& g, int gi, bool isPin, int need = 1, int exceptBi = -2) {
    const auto& st = g.stack[gi];
    // No limit but the lift: a stack goes as high as the lift can raise a piece over it
    if (goalTopIn(g, gi) + HALF_IN * (need - 1) + 1.0 > dr4bMaxLiftIn()) return false;
    // Read what the goal needs next: a pin on an empty goal, then the other kind than the top piece. Single
    // pieces other robots are already taking to it change that (each one flips the kind needed)
    bool wantPin = st.empty() ? true : !g.p[st.back()].isPin;
    int flips = 0;
    for (int j = 0; j < BOT_N; ++j) {
        const Bot& o = g.bots[j];
        if (j != exceptBi && o.on && o.goal == gi && o.carry >= 0 && o.rider < 0) ++flips;
    }
    if (exceptBi != BOT_N && g.pilot.on && g.pilot.goal == gi && g.pilot.carry >= 0 && g.pilot.rider < 0) ++flips;
    if (flips & 1) wantPin = !wantPin;
    return isPin == wantPin;
}

// A piece the robot may go for: loose and settled, not a cup with a pin standing in it, and not
// already wanted by another robot
inline bool botCanTake(const Game& g, int bi, int idx) {
    const Piece& pc = g.p[idx];
    if (!loose(pc) || pc.fz > 0.5 || idx == g.bots[bi].skip) return false;
    // Only pieces the claw can really get to: the claw sits 4 in behind the front bumper, so a piece
    // within about 3 in of a wall is out of reach, and so is one pressed against a goal or a Loader;
    // a piece still sliding or tipping isn't worth chasing yet
    const double fx = pc.x / PX_IN, fy = (720.0 - pc.y) / PX_IN;
    if (std::min({fx, fy, FIELD_IN - fx, FIELD_IN - fy}) < 3.2) return false;
    if (std::hypot(pc.vx, pc.vy) > 3.0 * PX_IN || pc.tilt > 0.2) return false;
    for (int gi = 0; gi < 9; ++gi)
        if (std::hypot(fx - GOAL_POS_IN[gi][0], fy - GOAL_POS_IN[gi][1]) < 4.6) return false;
    for (const LoaderDef& ld : LOADERS)
        if (std::hypot(fx - ld.x, fy - ld.y) < 4.5) return false;
    for (int j = 0; j < BOT_N; ++j)
        if (j != bi && g.bots[j].on && (g.bots[j].target == idx || g.bots[j].carry == idx)) return false;
    if (bi != BOT_N && g.pilot.on && (g.pilot.target == idx || g.pilot.carry == idx)) return false;
    return true;
}

// The pin standing in this cup, if any (taken and stacked together with it)
inline int botPinIn(const Game& g, int cup) {
    if (cup < 0 || g.p[cup].isPin) return -1;
    for (int j = 0; j < (int)g.p.size(); ++j)
        if (g.p[j].inCup == cup && loose(g.p[j])) return j;
    return -1;
}

// Loose pieces crowding a piece (not counting a cup it stands in or a pin standing in it): each one
// is likely to be shoved into the claw's way, so a clean pickup is preferred
inline int botClutter(const Game& g, int idx) {
    const Piece& pc = g.p[idx];
    int n = 0;
    for (int j = 0; j < (int)g.p.size(); ++j) {
        if (j == idx || !loose(g.p[j]) || j == pc.inCup || g.p[j].inCup == idx) continue;
        if (std::hypot(g.p[j].x - pc.x, g.p[j].y - pc.y) < 4.0 * PX_IN) ++n;
    }
    return n;
}

// Room to back up: the spot behind the back bumper (and its corners) is inside the field and clear
// of goals and Loaders by `margin` inches
inline bool botRearClear(const Game& g, const Bot& b, const std::array<GoalObstacle, 9>& goals, double margin) {
    (void)g;
    const double S = PX_IN, c = std::cos(b.h), s = std::sin(b.h), back = (BOT_HALF_IN + margin) * S;
    for (double side : {-0.8, 0.0, 0.8}) {
        const double px = b.x - c * back - s * side * BOT_HALF_IN * S, py = b.y - s * back + c * side * BOT_HALF_IN * S;
        if (px < 1.0 * S || px > 720.0 - 1.0 * S || py < 1.0 * S || py > 720.0 - 1.0 * S) return false;
        for (const auto& go : goals) if (std::hypot(px - go.x, py - go.y) < go.r + 1.0 * S) return false;
        for (const LoaderDef& ld : LOADERS) if (std::hypot(px - ld.x * S, py - (720.0 - ld.y * S)) < 3.5 * S) return false;
    }
    return true;
}
// Robots of the other alliance within r inches of a spot
inline int botOpponentsNear(const Game& g, Col team, double x, double y, double r) {
    int n = 0;
    for (const Bot& o : g.bots)
        if (o.on && o.team != team && std::hypot(o.x - x, o.y - y) < r * PX_IN) ++n;
    if (team == C_BLUE && !g.playerOff && std::hypot(g.pilot.x - x, g.pilot.y - y) < r * PX_IN && g.pilot.on) ++n;
    return n;
}
// Partners already taking a piece to this goal (they would queue at it)
inline int botTeammatesTo(const Game& g, int bi, Col team, int gi) {
    int n = 0;
    for (int j = 0; j < BOT_N; ++j)
        if (j != bi && g.bots[j].on && g.bots[j].team == team && g.bots[j].goal == gi && g.bots[j].carry >= 0) ++n;
    if (bi != BOT_N && team == C_RED && g.pilot.on && g.pilot.goal == gi && g.pilot.carry >= 0) ++n;
    return n;
}
// Where to drive with a piece for a goal: a lining-up point in front of it on its open side, then straight in
inline void botApproachGoal(const Game& g, Bot& b, const std::array<GoalObstacle, 9>& goals, double& tx, double& ty) {
    (void)g;
    const double S = PX_IN;
    const int key = -10 - b.goal;  // its own approach memory (pieces use their index)
    if (b.apTarget != key) { b.apTarget = key; b.apFinal = false; }
    const double gx = goals[b.goal].x, gy = goals[b.goal].y, fx = gx / S, fy = (720.0 - gy) / S;
    double ax = gx - b.x, ay = gy - b.y, al = std::max(1e-6, std::hypot(ax, ay));
    ax /= al; ay /= al;
    double nx = 0.0, ny = 0.0;  // open side: away from walls close to the goal
    const double R = 20.0;
    if (fx < R) nx += (R - fx) / R;
    if (FIELD_IN - fx < R) nx -= (R - (FIELD_IN - fx)) / R;
    if (fy < R) ny -= (R - fy) / R;
    if (FIELD_IN - fy < R) ny += (R - (FIELD_IN - fy)) / R;
    double dx = ax * 0.6 - nx * 1.4, dy = ay * 0.6 - ny * 1.4, dl = std::hypot(dx, dy);
    if (dl > 1e-6) { ax = dx / dl; ay = dy / dl; }
    const double back = (BOT_GRIP_IN + 11.0) * S;
    const double sx = gx - ax * back, sy = gy - ay * back, d = std::hypot(gx - b.x, gy - b.y);
    if (!b.apFinal && std::hypot(sx - b.x, sy - b.y) < 4.0 * S) b.apFinal = true;
    if (b.apFinal && d > back + 10.0 * S) b.apFinal = false;
    if (b.apFinal || d < BOT_GRIP_IN * S + 4.0 * S) { tx = gx; ty = gy; } else { tx = sx; ty = sy; }
}

// A little randomness in how a robot rates a job, so matches between equal robots don't repeat
inline double botJitter() { return 0.88 + 0.24 * (std::rand() / (double)RAND_MAX); }  // 0.88 .. 1.12

// Pick the cheapest piece-and-goal pair: drive to the piece, then on to an alliance goal it fits on.
// Pieces near the player's robot cost more, so the robots don't all fight over the same one.
inline void botPlan(Game& g, int bi, double plx, double ply) {
    Bot& b = g.bots[bi];
    const int gl[2] = {b.team == C_RED ? 0 : 2, b.team == C_RED ? 1 : 3};
    double best = 1e18;
    b.target = b.goal = -1;
    for (int i = 0; i < (int)g.p.size(); ++i) {
        if (!botCanTake(g, bi, i)) continue;
        const Piece& pc = g.p[i];
        double toPiece = std::hypot(pc.x - b.x, pc.y - b.y);
        double crowd = std::hypot(pc.x - plx, pc.y - ply) < 16.0 * PX_IN ? 40.0 * PX_IN : 0.0;
        crowd += botClutter(g, i) * 12.0 * PX_IN;
        for (int gi : gl) {
            if (!botFits(g, gi, pc.isPin)) continue;
            double cost = toPiece + std::hypot(pc.x - GOAL_POS_IN[gi][0] * PX_IN, pc.y - (720.0 - GOAL_POS_IN[gi][1] * PX_IN)) +
                          crowd + (pc.lying ? 10.0 * PX_IN : 0.0);
            if (cost < best) { best = cost; b.target = i; b.goal = gi; }
        }
    }
}

// Lift motor degrees that put the bottom of a held piece at this height (inverse of dr4bLiftIn)
inline double liftDegFor(double liftIn) {
    double lo = 0.0, hi = LIFT_MAX_DEG;
    for (int i = 0; i < 40; ++i) {
        double m = (lo + hi) / 2;
        if (dr4bLiftIn(m) < liftIn) lo = m; else hi = m;
    }
    return hi;
}

// ── Expert planning: everything is judged by what it does to the score ──
// A team's lead (its points minus the other alliance's) as the field stands
inline int teamMargin(Game& g, Col team) {
    int r, b;
    scoreField(g, r, b);
    return team == C_RED ? r - b : b - r;
}
// Endgame outlook: the points an alliance still needs to win beyond both alliances finishing in the Midfield (8 per robot, at most two a side).
// 0: it already wins that way, so it just drives to the Midfield; above 0: it is losing through the Midfield alone and has to score or take a Toggle
inline int endgameNeed(Game& g, Col team) {
    int r, b;
    int nr = g.playerOff ? 0 : 1, nb = 0;
    for (const Bot& bt : g.bots) if (bt.on) (bt.team == C_RED ? nr : nb)++;
    nr = std::min(nr, 2); nb = std::min(nb, 2);
    const int keepR = g.midRed, keepB = g.midBlue;
    g.midRed = nr; g.midBlue = nb;  // the Midfield Goal's yellows go to the alliance with more robots there, so count both finishing in it
    scoreField(g, r, b);
    g.midRed = keepR; g.midBlue = keepB;
    const int red = r + 8 * nr, blue = b + 8 * nb;
    return std::max(0, (team == C_RED ? blue - red : red - blue) + 1);
}
// Points a piece would add on top of a goal; a cup gets a little credit for setting up the next pin
// A yellow pin half is worth 10 to whoever owns that quadrant's Toggle at the end, so until the
// last 20 s the value is a blend of the score now and the score if the team owned every Toggle
inline double placeValue(Game& g, int gi, int idx, Col team, int idx2 = -1) {
    auto delta = [&]() {
        int before = teamMargin(g, team);
        g.stack[gi].push_back(idx);
        if (idx2 >= 0) g.stack[gi].push_back(idx2);
        int after = teamMargin(g, team);
        if (idx2 >= 0) g.stack[gi].pop_back();
        g.stack[gi].pop_back();
        return double(after - before);
    };
    const double now = delta();
    const auto saved = g.toggle;
    g.toggle = {{team, team, team, team}};
    const double owned = delta();
    g.toggle = saved;
    const double wNow = g.matchT < 20.0 ? 0.85 : 0.5;
    return wNow * now + (1.0 - wNow) * owned + (g.p[idx].isPin ? 0.0 : 4.0) + (idx2 >= 0 && !g.p[idx2].isPin ? 4.0 : 0.0);
}
// Points from turning Toggle k to the team's colour
inline double toggleValue(Game& g, int k, Col team) {
    if (g.toggle[k] == team) return 0.0;
    int before = teamMargin(g, team);
    Col old = g.toggle[k];
    g.toggle[k] = team;
    int after = teamMargin(g, team);
    g.toggle[k] = old;
    return after - before;
}
// Rolls to bring Toggle k to the team's colour (each ram rolls yellow -> red -> blue -> yellow)
inline int toggleRolls(Col now, Col want) { return now == want ? 0 : 1; }  // one roll, whichever way
// Toggle k: where the robot lines up (field inches) and the heading that puts its back, with the
// toggle arms, to that wall (0 left, 1 right, 2 bottom, 3 top)
inline void toggleStage(int k, double& fx, double& fy, double& h) {
    const double F = FIELD_IN, d = 22.0;
    switch (k) {
        case 0:  fx = d;     fy = F / 2; h = 0.0;       break;
        case 1:  fx = F - d; fy = F / 2; h = M_PI;      break;
        case 2:  fx = F / 2; fy = d;     h = -M_PI / 2; break;  // facing up the field (sim y up is -)
        default: fx = F / 2; fy = F - d; h = M_PI / 2;  break;
    }
}
inline bool botTaskTaken(const Game& g, int bi, int task, int k) {
    for (int j = 0; j < BOT_N; ++j)
        if (j != bi && g.bots[j].on && g.bots[j].task == task && g.bots[j].taskK == k) return true;
    if (bi != BOT_N && g.pilot.on && g.pilot.task == task && g.pilot.taskK == k) return true;
    return false;
}
// Best job per second of driving: a loose piece onto any goal, a piece from one of its Loaders, or a Toggle
// Goals a team may stack on: its own alliance goals, the neutral goals and the centre goal
inline bool goalAllowed(Col team, int gi) { return gi >= 4 || (team == C_RED ? gi <= 1 : (gi == 2 || gi == 3)); }
// The two robots of an alliance split the field so they don't fight over pieces, Loaders and goals: each works
// its own half (0 bottom, 1 top). Red: the partner the bottom, the player's robot (or its CPU copy) the top;
// blue: robot 1 the top (the Loader beside its goals), robot 2 the bottom. -1: no mate on the field, so everywhere.
inline int botSide(const Game& g, int bi) {
    if (bi == 0) return 0;
    if (bi == 3 || bi == BOT_N) return 1;
    if (bi == 1) return g.bots[2].on ? 1 : -1;
    return g.bots[1].on ? 0 : -1;
}
inline double botSideFactor(const Game& g, int bi, double fieldY) {
    const int s = botSide(g, bi);
    if (s < 0) return 1.0;
    return ((fieldY > FIELD_IN / 2) ? 1 : 0) == s ? 1.0 : 0.4;
}
// SCORE ON setting: the robots' own colour (alliance goals first) or yellows (neutral and centre goals first)
// SCORES YELLOW, or losing by a lot (posture 1): stack yellows and take Toggles
inline bool botYel(const Bot& b) { return b.scoreYel || b.posture == 1; }
inline double goalPref(int gi, bool yel) {
    const bool alliance = gi < 4;
    return !yel ? (alliance ? 1.35 : 0.85) : (alliance ? 0.8 : 1.45);
}
// What a robot concentrates on right now: its FOCUS 80 percent of the time and the other one for the last 4 s
// of every 20 s (staggered between robots). It only switches with an empty claw and not part way through a cycle.
constexpr double BOT_MODE_CYCLE_S = 20.0;
// Adapts to the score: losing by 25 or more with time left it goes risky (yellow stacks and Toggles), ahead by 25 or more it protects the lead
// (blocks opponents, guards a Toggle it owns). Back to normal once the gap is under 10.
inline void botUpdatePosture(Game& g, Bot& b, double dt) {
    if ((b.postureT -= dt) > 0.0) return;
    const int m = teamMargin(g, b.team), old = b.posture;
    int p = old;
    if (p == 0) { if (m <= -25 && g.matchT > 12.0) p = 1; else if (m >= 25 && g.matchT > 6.0) p = 2; }
    else if (p == 1) { if (m > -10 || g.matchT <= 8.0) p = 0; }
    else if (m < 10) p = 0;
    b.posture = p;
    b.postureT = p != old ? 5.0 : 0.5;  // a change holds for at least 5 s so it does not flip back and forth as a Toggle swings the margin
    if (p != old && std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "POSTURE t %.1f %s %s margin %d -> %s\n", g_matchLen - g.matchT, b.team == C_RED ? "red" : "blue", b.num, m, p == 1 ? "RISKY" : p == 2 ? "PROTECT" : "NORMAL");
}
inline int botModeNow(const Game& g, int bi, const Bot& b) {
    const double t = std::max(0.0, g_matchLen - g.matchT) + 5.0 * (bi + 1);
    const bool other = std::fmod(t, BOT_MODE_CYCLE_S) >= BOT_MODE_CYCLE_S * 0.8;
    const int alt = b.focusSet == FOCUS_DEFENSE ? FOCUS_SCORING : (b.focusSet == FOCUS_SCORING ? FOCUS_DEFENSE : FOCUS_DEFENSE);  // the other 20 percent: a scorer defends, a defender scores, TOGGLE/SCORING defends
    int want = other ? alt : b.focusSet;
    if (b.posture == 1) want = FOCUS_TOGGLE_SCORE;  // losing a lot: Toggles and yellows
    else if (b.posture == 2 && g.matchT < 0.85 * g_matchLen) want = FOCUS_DEFENSE;  // well ahead: protect it
    if (want == b.focus) return want;
    if (b.carry >= 0 || (b.focus == FOCUS_SCORING && b.cyc > 2)) return b.focus;
    return want;
}
inline void botPlanSmart(Game& g, int bi, double plx, double ply, double speedPx, const BotSkill& sk) {
    Bot& b = g.bots[bi];
    const double S = PX_IN;
    double best = 0.0;
    b.task = 0; b.taskK = -1; b.target = b.goal = -1; b.phase = 0; b.taskT = b.phaseT = 0.0;
    auto travel = [&](double x0, double y0, double x1, double y1) { return std::hypot(x1 - x0, y1 - y0) / speedPx; };
    auto gpx = [&](int gi) { return GOAL_POS_IN[gi][0] * S; };
    auto gpy = [&](int gi) { return 720.0 - GOAL_POS_IN[gi][1] * S; };
    // Net cost of a leg: points it risks through penalties (passing close to an opposing alliance goal or through the other alliance's Load Zone)
    auto riskCost = [&](double x0, double y0, double x1, double y1) {
        double cost = 0.0;
        const double sx = x1 - x0, sy = y1 - y0, sl = std::max(1e-6, sx * sx + sy * sy);
        auto segD = [&](double ox, double oy) { const double t = std::clamp(((ox - x0) * sx + (oy - y0) * sy) / sl, 0.0, 1.0); return std::hypot(ox - (x0 + sx * t), oy - (y0 + sy * t)); };
        for (int gi = 0; gi < 4; ++gi) if (!goalAllowed(b.team, gi) && segD(gpx(gi), gpy(gi)) < (BOT_HALF_IN + 2.8 + 8.0) * S) cost += 3.0;
        for (const LoaderDef& ld : LOADERS)
            if (ld.c != b.team && segD((ld.x < FIELD_IN / 2 ? LOADZONE_X / 2 : FIELD_IN - LOADZONE_X / 2) * S, 720.0 - ld.y * S) < (TILE_IN / 2 + BOT_HALF_IN + 4.0) * S) cost += 3.0;
        return cost;
    };
    int bestTask = 0, bestK = -1, bestT = -1, bestG = -1;
    // Match phase: before the endgame the Toggles are left alone (the other alliance would just roll
    // them back); in the endgame they come first, while pieces still count if there is time to place them
    const bool endgame = sk.endgame > 0.0 && g.matchT <= sk.endgame;
    double toggleWeight = endgame ? 2.0 : 0.1, pieceWeight = 1.0, loaderWeight = 1.0;
    const double timeLeft = g.matchT;
    // Losing through the Midfield alone in the endgame: only the points that close the gap count (the biggest job that fits wins, not the quickest)
    const int need = endgameNeed(g, b.team);
    const bool chase = need > 0 && need <= 60 && g.matchT <= std::max(sk.endgame, 25.0);
    switch (b.focus) {  // FOCUS setting
        case FOCUS_SCORING: pieceWeight = 1.2; loaderWeight = 1.1; toggleWeight = g.matchT <= 15.0 ? 2.0 : 0.02; break;
        case FOCUS_DEFENSE: pieceWeight = 0.5; toggleWeight = endgame ? 2.5 : 1.8; break;
        case FOCUS_TOGGLE_SCORE: pieceWeight = 1.1; loaderWeight = 1.0; toggleWeight = endgame ? 2.6 : 1.6; break;  // Toggles whenever one is worth it, and the rest of the time scoring
        default: break;
    }
    if (botYel(b)) toggleWeight = std::max(toggleWeight, endgame ? 2.6 : 2.0);  // a robot stacking yellows always tries to own its Toggle
    if (b.posture == 1) toggleWeight = std::max(toggleWeight, 2.2);  // losing by a lot: a Toggle swings its yellow halves
    if (chase) toggleWeight = pieceWeight = loaderWeight = 1.0;  // real points
    auto chaseRate = [&](double v, double tt) { return v * (v >= need ? 3.0 : 1.0) / (tt + 3.0); };
    for (int i = 0; i < (int)g.p.size(); ++i) {
        if (!botCanTake(g, bi, i)) continue;
        const Piece& pc = g.p[i];
        double extra = 2.5 + (pc.lying ? 0.5 : 0.0) + (std::hypot(pc.x - plx, pc.y - ply) < 16.0 * S ? 1.5 : 0.0) +
                       0.8 * botClutter(g, i) + 1.5 * botOpponentsNear(g, b.team, pc.x, pc.y, 15.0);
        for (int gi = 0; gi < 9; ++gi) {
            if (!goalAllowed(b.team, gi) || !botFits(g, gi, pc.isPin, botPinIn(g, i) >= 0 ? 2 : 1, bi)) continue;
            double v = placeValue(g, gi, i, b.team, botPinIn(g, i));  // a cup comes with the pin standing in it
            v -= riskCost(b.x, b.y, pc.x, pc.y) + riskCost(pc.x, pc.y, gpx(gi), gpy(gi));  // net of the penalty risk on the way
            if (v <= 0.5) continue;
            double tt = travel(b.x, b.y, pc.x, pc.y) + travel(pc.x, pc.y, gpx(gi), gpy(gi)) + extra +
                        1.0 * botOpponentsNear(g, b.team, gpx(gi), gpy(gi), 20.0) +  // a goal they guard costs time
                        1.2 * botTeammatesTo(g, bi, b.team, gi);                     // so does one a partner is taking a piece to
            if (tt > timeLeft) continue;  // can't be placed before the end
            double rate = pieceWeight * v / tt * botJitter() * goalPref(gi, botYel(b)) * botSideFactor(g, bi, GOAL_POS_IN[gi][1]) *
                          (0.6 + 0.4 * botSideFactor(g, bi, (720.0 - pc.y) / S));
            if (chase) rate = chaseRate(v, tt) * botJitter();
            if (rate > best) { best = rate; bestTask = 0; bestT = i; bestG = gi; bestK = -1; }
        }
    }
    for (int li = 0; li < 4 && sk.loaders; ++li) {
        if (LOADERS[li].c != b.team || g.loaderStack[li].empty() || botTaskTaken(g, bi, 1, li)) continue;
        const int f = g.loaderStack[li].front();
        const int f2 = g.loaderStack[li].size() >= 2 && g.p[g.loaderStack[li][1]].isPin != g.p[f].isPin ? g.loaderStack[li][1] : -1;
        const double ox = loaderOutX(li) * S, oy = 720.0 - LOADERS[li].y * S;
        for (int gi = 0; gi < 9; ++gi) {
            if (!goalAllowed(b.team, gi) || !botFits(g, gi, g.p[f].isPin, f2 >= 0 ? 2 : 1, bi)) continue;
            double v = placeValue(g, gi, f, b.team, f2);  // a pin and a cup come out together
            v -= riskCost(b.x, b.y, ox, oy) + riskCost(ox, oy, gpx(gi), gpy(gi));
            if (v <= 0.5) continue;
            double tt = travel(b.x, b.y, ox, oy) + travel(ox, oy, gpx(gi), gpy(gi)) + 4.5 + 1.2 * botTeammatesTo(g, bi, b.team, gi);
            if (tt > timeLeft) continue;
            double rate = loaderWeight * v * 1.15 / tt * botJitter() * goalPref(gi, botYel(b)) * botSideFactor(g, bi, LOADERS[li].y);  // a Loader piece comes out upright at a known spot: a surer pickup
            if (chase) rate = chaseRate(v, tt) * botJitter();
            if (rate > best) { best = rate; bestTask = 1; bestK = li; bestG = gi; bestT = -1; }
        }
    }
    for (int k = 0; k < 4; ++k) {
        if (botTaskTaken(g, bi, 2, k)) continue;
        double v = toggleWeight * toggleValue(g, k, b.team);
        if (v <= 0.5) continue;
        double fx, fy, h;
        toggleStage(k, fx, fy, h);
        double tt = travel(b.x, b.y, fx * S, 720.0 - fy * S) + 2.0 + 1.6 * toggleRolls(g.toggle[k], b.team);
        if (tt > timeLeft + 1.0) continue;
        double rate = chase ? chaseRate(v, tt) * botJitter() : v / tt * botJitter();
        if (rate > best) { best = rate; bestTask = 2; bestK = k; bestT = bestG = -1; }
    }
    b.task = bestTask; b.taskK = bestK; b.target = bestT; b.goal = bestG;
}

// The two goals each Loader serves (the nearest it can score on): red's near Loader the red alliance goals,
// red's far Loader the neutral goals of the top-left quadrant; blue the same turned half way round
inline std::array<int, 2> loaderGoals(int li) {
    switch (li) { case 0: return {0, 1}; case 1: return {5, 4}; case 2: return {6, 7}; default: return {2, 3}; }
}

// The CPU drive teams' match loads: blue keeps both blue Loaders stocked, the red partner's team the
// top-left red one, from their limited supply (MATCH_LOAD_PINS / CUPS). A pin with their colour on one
// half or a cup, whichever their goals need more, one every 3 s; none once the supply is gone.
inline void botsMatchLoads(Game& g, double dt, bool redLoads, bool blueLoads, double redReload = 3.0, double blueReload = 3.0) {
    for (int li = 0; li < 4; ++li) {
        // Each drive team stocks only the Loaders its own robots cycle (red: robot 0 the bottom one, the player's CPU copy or CPU
        // driver the top one; blue: robot 1 the top one, robot 2 the bottom one), so no match loads are wasted on an unused Loader
        const bool blueTeam = blueLoads && li >= 2 && (li == 3 ? g.bots[1].on : g.bots[2].on),
                   redTeam = redLoads && li < 2 && (li == 0 ? g.bots[0].on : true);
        if (!blueTeam && !redTeam) continue;
        const int a = blueTeam ? 1 : 0;
        auto& ls = g.loaderStack[li];
        if (ls.size() >= 2) { g.loadT[li] = 0.0; continue; }
        if ((g.loadT[li] += dt) < (ls.empty() ? (a == 0 ? redReload : blueReload) : 0.8)) continue;  // the second piece follows quickly: a pin and a cup together
        if (g.set.infiniteLoads) { g.loadPins[a] = std::max(g.loadPins[a], 1); g.loadCups[a] = std::max(g.loadCups[a], 1); }
        g.loadT[li] = 0.0;
        // The goal its robot uses next: the first of this Loader's goals with room for a pair. A pair always
        // leaves the top of a stack the same kind, so that goal wants the same pair every time: pin first on an
        // empty goal or one topped with a cup, cup first on one topped with a pin (a pin on its way there, such as
        // a Preload, counts as already on it)
        int goal = -1;
        for (int gi : loaderGoals(li)) if (goalTopIn(g, gi) + HALF_IN + 1.0 <= dr4bMaxLiftIn()) { goal = gi; break; }
        bool needPin = true;
        if (goal >= 0) {
            needPin = g.stack[goal].empty() || !g.p[g.stack[goal].back()].isPin;
            for (int k = 0; k <= BOT_N; ++k) {
                const Bot& bt = k == BOT_N ? g.pilot : g.bots[k];
                if (bt.on && bt.carry >= 0 && bt.goal == goal && g.p[bt.carry].isPin) needPin = false;
            }
        }
        bool pin = needPin;
        if (!ls.empty()) pin = !g.p[ls[0]].isPin;              // a Loader takes one of each
        if ((pin ? g.loadPins[a] : g.loadCups[a]) <= 0) pin = !pin;  // out of that kind: the other
        if ((pin ? g.loadPins[a] + g.loadYellow[a] : g.loadCups[a]) <= 0 || !loaderHasRoom(g, li, pin)) continue;
        const bool yellowPin = pin && g.loadPins[a] == 0;  // its coloured pins are used up: the yellow/yellow one
        if (yellowPin) --g.loadYellow[a];
        else --(pin ? g.loadPins[a] : g.loadCups[a]);
        Piece pc{pin, LOADERS[li].x * PX_IN, 720.0 - LOADERS[li].y * PX_IN, LOADERS[li].c, C_YEL};
        if (yellowPin) pc.top = C_YEL;
        pc.grayUp = true;
        pc.loader = li;
        pc.lz = LOADER_H + 1.0;
        ls.push_back((int)g.p.size());
        g.p.push_back(pc);
    }
}

// Where to drive for a piece: first to a lining-up point behind it, on its open side (away from
// walls, goals and Loaders close to it), then straight in so it meets the claw squarely
inline void botApproach(const Game& g, Bot& b, double& tx, double& ty) {
    const double S = PX_IN;
    const Piece& pc = g.p[b.target];
    if (b.apTarget != b.target) { b.apTarget = b.target; b.apFinal = false; }
    const double px = pc.x, py = pc.y, fx = px / S, fy = (720.0 - py) / S;
    double ax = px - b.x, ay = py - b.y;  // by default, from where the robot is
    double al = std::max(1e-6, std::hypot(ax, ay));
    ax /= al; ay /= al;
    double nx = 0.0, ny = 0.0;            // the piece's open side
    const double R = 16.0;
    if (fx < R) nx += (R - fx) / R;
    if (FIELD_IN - fx < R) nx -= (R - (FIELD_IN - fx)) / R;
    if (fy < R) ny -= (R - fy) / R;       // near the bottom wall: open toward the top (sim y up is -)
    if (FIELD_IN - fy < R) ny += (R - (FIELD_IN - fy)) / R;
    auto away = [&](double ox, double oy, double r) {
        double dx = px - ox, dy = py - oy, d = std::hypot(dx, dy);
        if (d < r * S && d > 1e-6) { nx += dx / d * (r * S - d) / (r * S); ny += dy / d * (r * S - d) / (r * S); }
    };
    for (int gi = 0; gi < 9; ++gi) away(GOAL_POS_IN[gi][0] * S, 720.0 - GOAL_POS_IN[gi][1] * S, 13.0);
    for (const LoaderDef& ld : LOADERS) away(ld.x * S, 720.0 - ld.y * S, 12.0);
    double dx = ax * 0.5 - nx * 1.5, dy = ay * 0.5 - ny * 1.5, dl = std::hypot(dx, dy);
    if (dl > 1e-6) { ax = dx / dl; ay = dy / dl; }
    const double back = (BOT_GRIP_IN + 9.0) * S;
    const double sx = px - ax * back, sy = py - ay * back;
    const double dPiece = std::hypot(px - b.x, py - b.y);
    if (!b.apFinal && std::hypot(sx - b.x, sy - b.y) < 3.5 * S) b.apFinal = true;
    if (b.apFinal && dPiece > back + 8.0 * S) b.apFinal = false;  // got pushed off the line: line up again
    if (b.apFinal || dPiece < BOT_GRIP_IN * S + 3.0 * S) { tx = px; ty = py; }
    else { tx = sx; ty = sy; }
}

// Watchdog: a robot that has stayed within 4 in for 4 s although it should be moving (not hesitating,
// docked at a Loader, holding the Midfield or defending) drops its job, backs out turning, and drives
// to an open spot for a couple of seconds before planning again, so it never sits stuck in a corner
inline void botWatchdog(Bot& b, double dt, bool defending) {
    const double S = PX_IN;
    b.wdT += dt;
    if (std::hypot(b.x - b.wdX, b.y - b.wdY) > 4.0 * S) { b.wdT = 0.0; b.wdX = b.x; b.wdY = b.y; }
    const bool resting = b.wait > 0.0 || b.backT > 0.0 || (b.task == 1 && b.phase >= 2) || b.task == 3 || defending;
    if (b.wdT < 4.0 || resting) return;
    b.wdT = 0.0;
    if (b.target >= 0) { b.skip = b.target; b.skipT = 8.0; }
    b.target = -1; b.task = 0; b.taskK = -1; b.phase = 0; b.cyc = 0;
    b.backT = 0.8;
    b.backTurn = (std::rand() & 1) ? 0.9 : -0.9;
    b.wanderT = 2.5;
    b.wanderX = (35.0 + 70.0 * (std::rand() / (double)RAND_MAX)) * S;
    b.wanderY = 720.0 - (35.0 + 70.0 * (std::rand() / (double)RAND_MAX)) * S;
}

// ── The SCORING focus: one fixed routine, over and over ──
// Forward to a Loader and take its pin and cup together, straight back out to a pivot point, turn, forward to
// a goal and let go, straight back to the same pivot, turn, forward to the Loader again. Every move is a
// straight line or a turn on the spot, nothing else. Now and then it goes for a Toggle. When the drive team
// has no pieces left it hands over to the normal planner.
enum CyclePhase { CY_START, CY_GOTO_R, CY_TURN_L, CY_DOCK, CY_REV_OUT, CY_TURN_G, CY_TO_GOAL, CY_REV_GOAL };
constexpr double CYC_PIVOT_IN = 13.0;  // the pivot is this far straight out from the dock (room to turn on the spot)

// The robot that uses the Loader next to its goals (red: bottom left; blue: top right); the other uses the far one
inline bool cycleIsNear(const Game& g, int bi, Col team) {
    if (team == C_RED) {
        const int order[3] = {0, 3, BOT_N};
        for (int k : order) {
            const bool on = k == BOT_N ? g.pilot.on : g.bots[k].on;
            if (on) return k == bi;
        }
        return true;
    }
    return g.bots[1].on ? bi == 1 : true;
}
struct CycGeo { double rx, ry, dockX, dockY, dockH; };  // pivot, robot centre at the dock, heading at the dock (sim px / radians)
inline CycGeo cycGeo(int li) {
    const double S = PX_IN;
    const bool left = LOADERS[li].x < FIELD_IN / 2;
    const double nx = left ? 1.0 : -1.0;                    // away from the wall
    const double dockIn = loaderOutX(li) + nx * BOT_GRIP_IN;  // the claw's grip point sits at the foot opening
    CycGeo c;
    c.dockX = dockIn * S;
    c.dockY = 720.0 - LOADERS[li].y * S;
    c.dockH = left ? M_PI : 0.0;
    c.rx = (dockIn + nx * CYC_PIVOT_IN) * S;
    // The Loaders sit about 11.4 in from the side walls, closer than the 12.7 in its corners sweep when it turns on the spot, so the pivot
    // is moved out to 14 in from the wall (the way in and out is a gentle slant that straightens up before the dock)
    c.ry = 720.0 - std::clamp(LOADERS[li].y, 14.0, FIELD_IN - 14.0) * S;
    return c;
}

// Returns true when the cycle drove this step. It either sets fwd / turn itself (direct) or leaves a
// target (tx, ty) for the normal steering; false hands the step to the normal planner.
inline bool botCycleStep(Game& g, int bi, Bot& b, const std::array<GoalObstacle, 9>& goals, double dt, bool pilot,
                         double& tx, double& ty, double& fwd, double& turn, bool& direct) {
    const double S = PX_IN, grip = BOT_GRIP_IN * S;
    const int ai = b.team == C_RED ? 0 : 1;
    auto bail = [&]() { direct = false; fwd = 0.0; turn = 0.0; return false; };  // handing over: leave the sticks to the normal planner
    // Yaw rate from the heading change: the same for the CPU robots and the CPU-driven player's robot
    if (dt > 0) b.cycW = 0.6 * b.cycW + 0.4 * wrapPi(b.h - b.cycPH) / dt;
    b.cycPH = b.h;
    b.cycT += dt;
    b.togCool += dt;
    if (b.cycOff > 0.0) { b.cycOff -= dt; return bail(); }
    const int li = b.team == C_RED ? (cycleIsNear(g, bi, b.team) ? 0 : 1) : (cycleIsNear(g, bi, b.team) ? 3 : 2);
    if (b.cycLi != li) { b.cycLi = li; b.cyc = CY_START; b.cycT = 0.0; }
    const CycGeo geo = cycGeo(li);
    auto& ls = g.loaderStack[li];
    auto setPhase = [&](int ph) { b.cyc = ph; b.cycT = 0.0; };
    auto turnPD = [&](double want) { return std::clamp(3.0 * wrapPi(want - b.h) - 0.45 * b.cycW, -1.0, 1.0); };
    // The heading that drives along the line (x0, y0) -> (x1, y1) (travel forward, or in reverse), turned
    // toward the line by the sideways error
    auto lineHeading = [&](double x0, double y0, double x1, double y1, bool reverse) {
        const double lh = std::atan2(y1 - y0, x1 - x0);
        const double lat = -(b.x - x0) * std::sin(lh) + (b.y - y0) * std::cos(lh);
        const double want = lh + std::atan2(-lat, 14.0 * S);
        return reverse ? wrapPi(want + M_PI) : want;
    };
    // Reverse speed: full, then braking to stop at the end of the leg (about 250 in/s^2)
    auto revSpeed = [&](double remPx) {
        const double d = std::max(remPx / S - 0.5, 0.0);
        return -std::clamp(std::sqrt(500.0 * d) / 76.6, 0.2, 1.0);
    };
    // The goal to take what it holds to: the first of this Loader's goals (the 90 degree route first) that has
    // room and takes the piece; a lone Preload with no such goal may go to any goal that takes it, nearest first
    auto chooseGoal = [&](double fx, double fy, bool any) {
        auto clearTo = [&](int gi) {  // no other goal close to the straight line from (fx, fy) to goal gi (the cycle drives straight legs)
            const double sx = goals[gi].x - fx, sy = goals[gi].y - fy, sl = std::max(1e-6, sx * sx + sy * sy);
            for (int k = 0; k < 9; ++k) {
                if (k == gi) continue;
                const double tt = std::clamp(((goals[k].x - fx) * sx + (goals[k].y - fy) * sy) / sl, 0.0, 1.0);
                if (std::hypot(goals[k].x - (fx + sx * tt), goals[k].y - (fy + sy * tt)) < (BOT_HALF_IN + 2.8 + 3.0) * S) return false;
            }
            return true;
        };
        const int need = b.rider >= 0 ? 2 : 1;
        if (botYel(b)) {  // SCORES yellows: the nearest neutral or centre goal with a clear straight line from here
            int bestY = -1;
            double bestYd = 1e18;
            for (int gi = 4; gi < 9; ++gi) {
                if (!botFits(g, gi, g.p[b.carry].isPin, need, bi)) continue;
                bool clear = true;
                const double sx = goals[gi].x - fx, sy = goals[gi].y - fy, sl = std::max(1e-6, sx * sx + sy * sy);
                for (int k = 0; k < 9 && clear; ++k) {
                    if (k == gi) continue;
                    const double tt = std::clamp(((goals[k].x - fx) * sx + (goals[k].y - fy) * sy) / sl, 0.0, 1.0);
                    if (std::hypot(goals[k].x - (fx + sx * tt), goals[k].y - (fy + sy * tt)) < (BOT_HALF_IN + 2.8 + 3.0) * S) clear = false;
                }
                const double d = std::sqrt(sl);
                if (clear && d < bestYd) { bestYd = d; bestY = gi; }
            }
            if (bestY >= 0) return bestY;
        }
        for (int gi : loaderGoals(li)) {
            if (!goalAllowed(b.team, gi) || !botFits(g, gi, g.p[b.carry].isPin, need, bi) || !clearTo(gi)) continue;
            return gi;
        }
        if (!any) return -1;
        int best = -1;
        double bestD = 1e18;
        for (int gi = 0; gi < 8; ++gi) {
            if (!goalAllowed(b.team, gi) || !botFits(g, gi, g.p[b.carry].isPin, need, bi) || !clearTo(gi)) continue;
            const double d = std::hypot(goals[gi].x - fx, goals[gi].y - fy);
            if (d < bestD) { bestD = d; best = gi; }
        }
        return best;
    };
    auto startLeg = [&](double x1, double y1) { b.cycX0 = b.x; b.cycY0 = b.y; b.cycX1 = x1; b.cycY1 = y1; };
    direct = true;
    fwd = turn = 0.0;

    // ── Holding something with no plan: the Preload at the start, or a piece the normal planner left us with ──
    if (b.cyc == CY_START && b.carry >= 0) {
        const int gi = chooseGoal(b.x, b.y, true);
        if (gi < 0) return bail();
        b.goal = b.cycGoal = gi;
        b.liftTarget = liftDegFor(goalTopIn(g, gi) + 0.8);
        setPhase(CY_TURN_G);
    }
    if (b.carry < 0 && (b.cyc == CY_START || b.cyc == CY_GOTO_R || b.cyc == CY_TURN_L)) {
        // Nothing left to load: the normal planner takes over (floor pieces)
        const bool supply = g.set.infiniteLoads || (g.loadPins[ai] + g.loadYellow[ai] + g.loadCups[ai]) > 0;
        if (!supply && ls.empty()) { b.cycOff = 2.0; return bail(); }
        // A Toggle trip now and then (at least 30 s apart), when one is worth enough points
        if (b.togCool > (botYel(b) ? 10.0 : 30.0) && g.matchT > 14.0 && g.matchT < g_matchLen - 15.0) {
            int bestK = -1;
            double bestV = botYel(b) ? 2.0 : 6.0;
            for (int k = 0; k < 4; ++k) {
                if (botTaskTaken(g, bi, 2, k)) continue;
                const double v = toggleValue(g, k, b.team);
                if (v > bestV) { bestV = v; bestK = k; }
            }
            if (bestK >= 0) {
                b.task = 2; b.taskK = bestK; b.phase = 0; b.taskT = 0.0; b.togCool = 0.0;
                b.cyc = CY_START;
                return bail();
            }
            b.togCool = 20.0;  // nothing worth it: look again in 10 s
        }
        if (b.cyc == CY_START) setPhase(CY_GOTO_R);
    }
    b.liftTarget = b.carry >= 0 ? b.liftTarget : 0.0;

    switch (b.cyc) {
        case CY_GOTO_R: {            // get to the pivot in front of the Loader (normal steering)
            direct = false;
            tx = geo.rx; ty = geo.ry;
            if (std::hypot(tx - b.x, ty - b.y) < 4.0 * S) setPhase(CY_TURN_L);
            else if (b.cycT > 12.0) setPhase(CY_START);
            return true;
        }
        case CY_TURN_L: {            // turn on the spot to face the dock
            const double want = std::atan2(geo.dockY - b.y, geo.dockX - b.x);
            turn = turnPD(want);
            if ((std::fabs(wrapPi(want - b.h)) < 0.07 && std::fabs(b.cycW) < 1.0) || b.cycT > 2.5) {
                startLeg(geo.dockX, geo.dockY);
                setPhase(CY_DOCK);
            }
            return true;
        }
        case CY_DOCK: {              // straight in until the Loader stops it, then take what comes out
            const double D = std::hypot(geo.dockX - b.x, geo.dockY - b.y);
            const double gxp = b.x + std::cos(b.h) * grip, gyp = b.y + std::sin(b.h) * grip;
            const double dO = std::hypot(gxp - loaderOutX(li) * S, gyp - (720.0 - LOADERS[li].y * S));
            if (D < 24.0 * S) g.botAtLoader[li] = true;  // its clear tube starts rising as the robot closes in
            // steer onto the dock's own axis (a line straight out from the Loader), not the slanted line from the pivot, so it arrives square
            turn = std::clamp(3.0 * wrapPi(lineHeading(geo.dockX - std::cos(geo.dockH) * 30.0 * S, geo.dockY, geo.dockX, geo.dockY, false) - b.h) - 0.35 * b.cycW, -0.6, 0.6);
            fwd = D > 15.0 * S ? 1.0 : (D > 5.0 * S ? 0.6 : 0.35);
            if (D < 0.6 * S) fwd = 0.0;
            if (b.carry < 0) {
                const int team = ai;
                const bool out = !ls.empty() && g.p[ls.front()].slide >= 0.95 && dO < 2.0 * S;
                const bool second = ls.size() == 1 && (g.p[ls[0]].isPin ? g.loadCups[team] : g.loadPins[team]) > 0;
                if (out && !pilot) {
                    if (second && (b.dockWait += dt) < 1.5) { fwd = 0.0; }  // the pair comes out together: wait for the second piece
                    else {
                        const int idx = ls.front();
                        ls.erase(ls.begin());
                        Piece& pc = g.p[idx];
                        pc.loader = -1; pc.slide = 0.0; pc.lz = pc.lzv = 0.0;
                        pc.carrier = bi; pc.lying = false; pc.vx = pc.vy = 0.0;
                        b.carry = idx;
                        ++b.loaded;
                        if (!ls.empty() && g.p[ls.front()].isPin != pc.isPin) {  // the other piece, nested on it
                            const int r = ls.front();
                            ls.erase(ls.begin());
                            Piece& rp = g.p[r];
                            rp.loader = -1; rp.slide = 0.0; rp.lz = rp.lzv = 0.0;
                            rp.carrier = bi; rp.lying = false; rp.vx = rp.vy = 0.0;
                            b.rider = r;
                            ++b.loaded;
                        }
                        b.clawDeg += 120.0;
                        b.dockWait = 0.0;
                    }
                } else if (!out && D < 2.0 * S) {
                    fwd = 0.0;
                }
                if (b.carry < 0 && b.cycT > 8.0) { b.cycOff = 6.0; setPhase(CY_START); return bail(); }  // no pieces coming: normal planner for a while
            }
            if (b.carry >= 0) {      // got it: pick the goal and back out
                b.dockWait = 0.0;
                const int gi = chooseGoal(geo.rx, geo.ry, false);
                if (gi < 0) { setPhase(CY_START); return bail(); }
                b.goal = b.cycGoal = gi;
                b.liftTarget = liftDegFor(goalTopIn(g, gi) + 0.8);
                startLeg(geo.rx, geo.ry);
                setPhase(CY_REV_OUT);
            }
            return true;
        }
        case CY_REV_OUT: {           // straight back to the pivot; once clear of the Loader, turn toward the goal while backing
            const double len = std::max(std::hypot(b.cycX1 - b.cycX0, b.cycY1 - b.cycY0), 1.0);
            const double along = (b.x - b.cycX0) * (b.cycX1 - b.cycX0) / len + (b.y - b.cycY0) * (b.cycY1 - b.cycY0) / len;
            const double rem = len - along;
            if (along < 6.0 * S) g.botAtLoader[li] = true;
            double want = lineHeading(b.cycX0, b.cycY0, b.cycX1, b.cycY1, true);
            turn = turnPD(want);
            fwd = revSpeed(rem);
            if (rem < 1.5 * S || b.cycT > 3.0) setPhase(CY_TURN_G);
            return true;
        }
        case CY_TURN_G: {            // turn on the spot to face the goal
            if (b.goal < 0) { setPhase(CY_START); return bail(); }
            const double want = std::atan2(goals[b.goal].y - b.y, goals[b.goal].x - b.x);
            turn = turnPD(want);
            if ((std::fabs(wrapPi(want - b.h)) < 0.06 && std::fabs(b.cycW) < 1.0) || b.cycT > 2.5) {
                b.cycRx = b.x; b.cycRy = b.y;
                startLeg(goals[b.goal].x, goals[b.goal].y);
                setPhase(CY_TO_GOAL);
            }
            return true;
        }
        case CY_TO_GOAL: {           // straight at the goal; the alignment guide and the claw do the rest
            if (b.carry < 0) {       // let go: back along the same line
                startLeg(b.cycRx, b.cycRy);
                b.liftTarget = 0.0;
                setPhase(CY_REV_GOAL);
                return true;
            }
            if (b.goal < 0) { setPhase(CY_START); return bail(); }
            const double D = std::hypot(goals[b.goal].x - b.x, goals[b.goal].y - b.y) - grip;
            turn = std::clamp(3.0 * wrapPi(lineHeading(b.cycX0, b.cycY0, goals[b.goal].x, goals[b.goal].y, false) - b.h) - 0.35 * b.cycW, -0.6, 0.6);
            fwd = D > 20.0 * S ? 1.0 : 0.7;
            if (D < 14.0 * S && b.liftDeg < b.liftTarget - 30.0) fwd = 0.1;  // wait for the lift
            if (b.cycT > 5.0) setPhase(CY_START);
            return true;
        }
        case CY_REV_GOAL: {          // straight back to the pivot; once clear of the goal, turn toward the Loader while backing
            b.liftTarget = 0.0;
            const double len = std::max(std::hypot(b.cycX1 - b.cycX0, b.cycY1 - b.cycY0), 1.0);
            const double along = (b.x - b.cycX0) * (b.cycX1 - b.cycX0) / len + (b.y - b.cycY0) * (b.cycY1 - b.cycY0) / len;
            const double rem = len - along;
            double want = lineHeading(b.cycX0, b.cycY0, b.cycX1, b.cycY1, true);
            turn = turnPD(want);
            fwd = revSpeed(rem);
            if (rem < 1.5 * S || b.cycT > 3.0) setPhase(std::hypot(geo.rx - b.x, geo.ry - b.y) > 30.0 * S ? CY_GOTO_R : CY_TURN_L);
            return true;
        }
        default:
            setPhase(CY_START);
            return bail();
    }
}

inline bool inLoadZone(double x, double y, double h, double hl, double hw2, Col zone);
// A piece the robot can't use: pick a spot to drop it that is out of its way (away from the lines between its Loader and goals, from
// walls, goals, Loaders, the other alliance's Load Zone, other robots and loose pieces) and not far from where it is, and set off for it
inline void botStartDump(Game& g, Bot& b, int bi) {
    if (b.dropT > 0.0) return;  // the CPU-driven player's robot is already letting go
    const double S = PX_IN;
    struct Seg { double x0, y0, x1, y1; };
    std::vector<Seg> path;
    {
        const int li = b.team == C_RED ? (cycleIsNear(g, bi, b.team) ? 0 : 1) : (cycleIsNear(g, bi, b.team) ? 3 : 2);
        const CycGeo geo = cycGeo(li);
        for (int gi : loaderGoals(li)) path.push_back({geo.dockX, geo.dockY, GOAL_POS_IN[gi][0] * S, 720.0 - GOAL_POS_IN[gi][1] * S});
        path.push_back({b.x, b.y, geo.dockX, geo.dockY});
    }
    auto segD = [&](const Seg& s, double px, double py) {
        const double sx = s.x1 - s.x0, sy = s.y1 - s.y0, sl = std::max(1e-6, sx * sx + sy * sy);
        const double t = std::clamp(((px - s.x0) * sx + (py - s.y0) * sy) / sl, 0.0, 1.0);
        return std::hypot(px - (s.x0 + sx * t), py - (s.y0 + sy * t)) / S;
    };
    double best = -1e18, bx = 0.0, by = 0.0;
    bool found = false;
    for (double fx = 14.0; fx <= FIELD_IN - 14.0; fx += 6.0)
        for (double fy = 14.0; fy <= FIELD_IN - 14.0; fy += 6.0) {
            const double px = fx * S, py = 720.0 - fy * S;
            const double d0 = std::hypot(px - b.x, py - b.y) / S;
            if (d0 < 14.0 || d0 > 55.0) continue;                  // it has to move away, but not across the field
            bool bad = false;
            for (int gi = 0; gi < 9 && !bad; ++gi) if (std::hypot(fx - GOAL_POS_IN[gi][0], fy - GOAL_POS_IN[gi][1]) < 17.0) bad = true;
            for (const LoaderDef& ld : LOADERS) if (!bad && std::hypot(fx - ld.x, fy - ld.y) < 24.0) bad = true;  // Loaders and their Load Zones
            for (int j = 0; j < BOT_N && !bad; ++j) if (j != bi && g.bots[j].on && std::hypot(px - g.bots[j].x, py - g.bots[j].y) < 16.0 * S) bad = true;
            if (bad) continue;
            double pathD = 30.0;
            for (const Seg& s : path) pathD = std::min(pathD, segD(s, px, py));
            double pieceD = 12.0;
            for (const Piece& pc : g.p) if (loose(pc)) pieceD = std::min(pieceD, std::hypot(px - pc.x, py - pc.y) / S);
            const double wall = std::min(std::min(fx, fy), std::min(FIELD_IN - fx, FIELD_IN - fy));
            const double score = pathD + 0.4 * std::min(wall, 30.0) + 0.3 * pieceD - 0.25 * d0;
            if (score > best) { best = score; bx = px; by = py; found = true; }
        }
    if (!found) return;
    b.dumping = true; b.dumpX = bx; b.dumpY = by; b.dumpT = 0.0;
    if (std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "DUMP t %.1f bot %d from (%.0f,%.0f) to (%.0f,%.0f)\n", g_matchLen - g.matchT, bi, b.x / S, (720.0 - b.y) / S, bx / S, (720.0 - by) / S);
    b.cyc = 0; b.cycT = 0.0; b.cycOff = 0.0; b.task = 0; b.target = -1; b.goal = -1; b.wait = 0.0; b.backT = 0.0;
}
struct BotCmd { double fwd = 0.0, turn = 0.0; bool defending = false, skip = false; };
inline std::array<int, 2> botGoalsOf(Col t) { return {t == C_RED ? 0 : 2, t == C_RED ? 1 : 3}; }
// What a robot does this step (its job, where it steers, the lift), as stick commands. bi is its
// index in Game::bots, or BOT_N for the CPU driving the player's robot (pilot: it drives through the
// robot code, and its claw takes pieces by itself). plx, ply: the player's robot, to steer round.
inline BotCmd botDecide(Game& g, int bi, Bot& b, const BotSkill& sk, const std::array<GoalObstacle, 9>& goals,
                        double dt, double plx, double ply, bool playerHolding, bool pilot) {
    const double S = PX_IN, grip = BOT_GRIP_IN * S, TOP = g_topIps * S;
    const double liftIn = dr4bLiftIn(b.liftDeg);
    const double c0 = std::cos(b.h), s0 = std::sin(b.h);
    double fwd = 0.0, turn = 0.0;  // the computer's sticks, -1 to 1
    bool defending = false, direct = false;

    botUpdatePosture(g, b, dt);
    if (b.pinT > 3.0 && b.backT <= 0) { if (std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "PIN-RELEASE t %.1f %s %s\n", g_matchLen - g.matchT, b.team == C_RED ? "red" : "blue", b.num); b.backT = 1.0; b.pinT = 0.0; b.restT = 5.0; b.defendT = 0.0; }  // never hold an opponent against something: let go well before the 5 s call
    if (b.backT > 0) {                     // backing away (after placing, or out of a jam)
        b.backT -= dt;
        if (botRearClear(g, b, goals, 4.0)) { fwd = -0.5; turn = b.backTurn; }
        else { fwd = 0.0; turn = b.backTurn != 0.0 ? b.backTurn : 0.9; }  // no room behind: turn on the spot instead
        direct = true;
    } else if (b.wait > 0) {               // the driver hesitates
        b.wait -= dt;
        direct = true;
    } else {
        double tx = b.x, ty = b.y;
        int homeGoal = -1;
        // A piece it can't use: drive to the dump spot (the normal steering keeps clear of goals, walls and robots on the way), let it go there
        bool dumpingNow = b.dumping && b.carry >= 0;
        if (b.dumping && b.carry < 0) b.dumping = false;
        if (dumpingNow) {
            b.liftTarget = 0.0; b.task = 0; b.target = -1; b.goal = -1;
            tx = b.dumpX; ty = b.dumpY;
            if ((b.dumpT += dt) > 8.0 || std::hypot(tx - b.x, ty - b.y) < grip + 2.5 * S) {
                b.dumping = false; b.dumpT = 0.0; dumpingNow = false;
                if (std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "DROP t %.1f bot %d at (%.0f,%.0f) spot (%.0f,%.0f)\n", g_matchLen - g.matchT, bi, b.x / S, (720.0 - b.y) / S, tx / S, (720.0 - ty) / S);
                if (bi == BOT_N) b.dropT = 1.5;  // its piece is in the real claw: the claw runs out (pilotUpdate)
                else botRelease(g, b);
                b.backT = 0.8; b.backTurn = 0.0;
            }
        }
        // Hard and expert: the first blue robot defends a red goal the player is bringing a piece to (5 s at most, then 8 s off)
        // Defense (the DEFENSE strategy): when an
        // opponent carries a piece toward one of its alliance goals, park 14 in in front of that goal
        // (5 s at most, then 8 s off, so it never pins anyone for long)
        const bool defender = b.focus == FOCUS_DEFENSE;  // only when its strategy is DEFENSE (fair to both alliances)
        bool guarding = false;
        if (defender && b.carry < 0 && b.task == 0) {
            b.restT = std::max(0.0, b.restT - dt);
            // The biggest threat: an opponent carrying pieces, the more it carries and the nearer it is to
            // the goal it is heading for (and to this robot), the bigger. Its own goal if known, else the
            // nearest goal it may use.
            double bestThreat = 0.0, bestD = 1.0, ox = 0, oy = 0;
            int gd = -1;
            auto consider = [&](double x, double y, double vx, double vy, int pieces, int goal, Col theirs) {
                if (pieces <= 0 || theirs == b.team) return;
                if (goal < 0) {
                    double nd = 1e18;
                    for (int gi = 0; gi < 9; ++gi)
                        if (goalAllowed(theirs, gi)) {
                            double d = std::hypot(x - goals[gi].x, y - goals[gi].y);
                            if (d < nd) { nd = d; goal = gi; }
                        }
                }
                const double d = std::hypot(x - goals[goal].x, y - goals[goal].y);
                if (d > 70.0 * S || d < 1.0) return;
                const double threat = pieces * (1.0 - d / (70.0 * S)) / (1.0 + std::hypot(b.x - x, b.y - y) / (60.0 * S));
                if (threat > bestThreat) {
                    bestThreat = threat; gd = goal; bestD = d;
                    ox = x + vx * 0.5; oy = y + vy * 0.5;  // where it will be in half a second
                }
            };
            consider(plx, ply, 0.0, 0.0, playerHolding ? (g.rider >= 0 ? 2 : 1) : 0, -1, C_RED);  // the player's robot (red)
            for (int j = 0; j < BOT_N; ++j) {
                const Bot& o = g.bots[j];
                if (j != bi && o.on) consider(o.x, o.y, o.vx, o.vy, (o.carry >= 0) + (o.rider >= 0), o.carry >= 0 ? o.goal : -1, o.team);
            }
            // Block only a race it can win: it must reach the blocking spot before the carrier reaches its
            // goal. Otherwise it keeps scoring (a defender that arrives late just loses its own points).
            const double spd = g_topIps * S * sk.stick * 0.7;
            bool block = false;
            if (gd >= 0 && b.restT <= 0) {
                const double back = std::max(14.0 * S, bestD * 0.45);
                const double bx2 = goals[gd].x + (ox - goals[gd].x) / bestD * back, by2 = goals[gd].y + (oy - goals[gd].y) / bestD * back;
                const double mine = std::hypot(bx2 - b.x, by2 - b.y) / spd + 0.4, theirs = (bestD - back) / spd + 1.0;
                const bool close = std::hypot(ox - b.x, oy - b.y) < 16.0 * S;
                if (mine + 0.5 < theirs || close || b.defendT > 0.0 || bestThreat > 0.0) {  // a defender always goes for the biggest threat: an annoyance, not a gamble
                    block = true;
                    defending = true;
                    // Near the carrier: drive into it to knock it off its line; else get on its path, at least
                    // 14 in out from the goal (a robot never touches an opposing Alliance Goal, <SG9>)
                    if (close) { tx = ox; ty = oy; }
                    else { tx = bx2; ty = by2; }
                    if ((b.defendT += dt) > 9.0) { b.defendT = 0.0; b.restT = 2.0; }  // 9 s at most, then 2 s off
                }
            }
            int togK = -1;  // DEFENSE/TOGGLE: with nobody to block, turn a Toggle first
            if (!block) {
                double bestTv = 0.5;
                for (int k = 0; k < 4; ++k) {
                    if (botTaskTaken(g, bi, 2, k)) continue;
                    const double tv = toggleValue(g, k, b.team);
                    if (tv > bestTv) { bestTv = tv; togK = k; }
                }
                if (togK >= 0) { b.task = 2; b.taskK = togK; b.phase = 0; b.taskT = 0.0; }
            }
            if (!block && togK < 0 && b.posture == 2 && b.restT <= 0) {  // well ahead: stand in front of a Toggle its alliance owns when an opponent is heading that way
                double bestGd = 1e18, gx2 = 0, gy2 = 0;
                for (int k = 0; k < 4; ++k) {
                    if (g.toggle[k] != b.team) continue;
                    double fx, fy, sh;
                    toggleStage(k, fx, fy, sh);
                    const double sx2 = fx * S, sy2 = 720.0 - fy * S;
                    double nearest = std::hypot(plx - sx2, ply - sy2);
                    for (int j = 0; j < BOT_N; ++j) if (j != bi && g.bots[j].on && g.bots[j].team != b.team) nearest = std::min(nearest, std::hypot(g.bots[j].x - sx2, g.bots[j].y - sy2));
                    if (b.team == C_RED || b.team == C_BLUE) { if (nearest < 80.0 * S && nearest < bestGd) { bestGd = nearest; gx2 = sx2; gy2 = sy2; } }
                }
                if (bestGd < 1e17) {
                    block = true; defending = true; guarding = true; tx = gx2; ty = gy2;
                    if ((b.defendT += dt) > 9.0) { b.defendT = 0.0; b.restT = 2.0; }
                }
            }
            if (!block && togK < 0 && b.restT <= 0) {  // nobody is carrying and no Toggle to turn: shadow the nearest opponent and stay in its way
                double nd = 1e18, sx = 0, sy = 0;
                auto shadow = [&](double x, double y, Col t, double h) {
                    if (t == b.team || inLoadZone(x, y, h, (BOT_HALF_IN + 14.0) * S, (BOT_HALF_IN + 14.0) * S, t)) return;  // nothing near its own Load Zone (a robot there is protected)
                    const double d = std::hypot(x - b.x, y - b.y);
                    if (d < nd) { nd = d; sx = x; sy = y; }
                };
                shadow(plx, ply, C_RED, 0.0);
                for (int j = 0; j < BOT_N; ++j) if (j != bi && g.bots[j].on) shadow(g.bots[j].x, g.bots[j].y, g.bots[j].team, g.bots[j].h);
                if (nd < 1e17) {
                    block = true; defending = true; tx = sx; ty = sy;
                    if ((b.defendT += dt) > 9.0) { b.defendT = 0.0; b.restT = 2.0; }
                }
            }
            if (!block) b.defendT = 0.0;
            if (guarding && std::hypot(tx - b.x, ty - b.y) < 3.0 * S) { direct = true; fwd = 0.0; turn = 0.0; }  // parked on the Toggle's approach
        }
        // Endgame (the last 10 s, REGIONAL and up): finish in the Midfield, lift down (<SG12>, <SC6>).
        // Red takes the west half, blue the east; it sets off in time to get there.
        if (sk.endgame > 0.0 && !dumpingNow) {
            const double mfx = (b.team == C_RED ? 70.2 - 15.0 : 70.2 + 15.0) * S;
            const bool upper = b.team == C_RED ? bi == 0 : bi == 1;  // two spots per alliance, blue's the mirror image of red's
            const double mfy = 720.0 - (70.2 + (upper ? 8.0 : -8.0)) * S;
            const double eta = std::hypot(mfx - b.x, mfy - b.y) / (g_topIps * S * sk.stick * 0.6) + 1.5;
            if (g.matchT <= std::max(10.0, eta) && b.task != 3) {
                // Cost against benefit: the Midfield is a sure 8, but when the alliance loses even with both robots there, it
                // has to score or take a Toggle instead, if a job still fits in the time left
                bool chase = false;
                const int need = endgameNeed(g, b.team);
                if (need > 0 && need <= 60 && g.matchT > 1.5) {
                    const double spd = TOP * sk.stick * 0.6;
                    auto gpt = [&](int gi) { return std::hypot(GOAL_POS_IN[gi][0] * S - b.x, 720.0 - GOAL_POS_IN[gi][1] * S - b.y); };
                    double tneed = -1.0;
                    if (b.carry >= 0 && b.goal >= 0) tneed = gpt(b.goal) / spd + 3.0;
                    else if (b.task == 2 && b.taskK >= 0) { double fx, fy, sh; toggleStage(b.taskK, fx, fy, sh); tneed = std::hypot(fx * S - b.x, 720.0 - fy * S - b.y) / spd + 3.5; }
                    else if (b.task == 1 || (b.task == 0 && b.target >= 0 && b.goal >= 0)) tneed = 0.0;  // on a job already; the planner only picked it if it fits
                    else if (b.carry < 0 && sk.smart) {  // idle: look for a job that closes the gap
                        botPlanSmart(g, bi, plx, ply, spd, sk);
                        if (b.task != 0 || b.target >= 0) tneed = 0.0;
                    }
                    chase = tneed >= 0.0 && tneed < g.matchT - 0.3;
                }
                if (!chase) { b.task = 3; b.target = -1; b.phase = 0; }
            }
            if (b.task == 3 && g.matchT > 20.0) b.task = 0;  // a new match: back to work
            if (b.task == 3 && b.carry >= 0 && b.goal >= 0 && g.matchT > 3.0) {  // the score turned against its alliance while it holds a piece it can still place: place it first
                const int nd = endgameNeed(g, b.team);
                const double dg = std::hypot(GOAL_POS_IN[b.goal][0] * S - b.x, 720.0 - GOAL_POS_IN[b.goal][1] * S - b.y);
                if (nd > 0 && nd <= 60 && dg / (TOP * sk.stick * 0.6) + 3.0 < g.matchT - 0.3) b.task = 0;
            }
            if (b.task == 3) { tx = mfx; ty = mfy; }
        }
        if (b.task == 3 && !defending) {
            b.liftTarget = 0.0;
            // DEFENSE in the Endgame: push the nearest opponent that is in the Midfield back out of it
            int pushJ = -1;
            double pushD = 1e18, pushX = 0, pushY = 0;
            if (b.focus == FOCUS_DEFENSE) {
                auto consider = [&](double x, double y, Col t) {
                    if (t == b.team || std::fabs(x - 360.0) + std::fabs(y - 360.0) > MIDFIELD_PX + 6.0 * S) return;
                    double d = std::hypot(x - b.x, y - b.y);
                    if (d < pushD) { pushD = d; pushX = x; pushY = y; pushJ = 1; }
                };
                consider(plx, ply, C_RED);
                for (int j = 0; j < BOT_N; ++j) if (j != bi && g.bots[j].on) consider(g.bots[j].x, g.bots[j].y, g.bots[j].team);
            }
            if (pushJ >= 0) { tx = pushX; ty = pushY; }
            else if (std::hypot(tx - b.x, ty - b.y) < 3.0 * S) { direct = true; fwd = 0.0; turn = 0.0; }  // hold the spot
        } else if (dumpingNow) {
            // steered to the dump spot below
        } else if (b.wanderT > 0.0 && !defending) {  // was stuck: get out into open field first
            b.wanderT -= dt;
            tx = b.wanderX; ty = b.wanderY;
        } else if (!defending && b.focus == FOCUS_SCORING && b.task == 0 &&
                   botCycleStep(g, bi, b, goals, dt, pilot, tx, ty, fwd, turn, direct)) {
            // the SCORING routine drove this step
        } else if (!defending) {
            if (b.carry >= 0) {
                b.task = 0;
                if (b.goal < 0 || !botFits(g, b.goal, g.p[b.carry].isPin, b.rider >= 0 ? 2 : 1, bi)) {  // its goal changed (or a partner is putting the same kind on it): find another
                    b.goal = -1;
                    double bestV = 0.0;
                    if (sk.smart) {
                        for (int gi = 0; gi < 9; ++gi)
                            if (goalAllowed(b.team, gi) && botFits(g, gi, g.p[b.carry].isPin, b.rider >= 0 ? 2 : 1, bi)) {
                                double v = placeValue(g, gi, b.carry, b.team) * goalPref(gi, botYel(b));
                                if (v > bestV) { bestV = v; b.goal = gi; }
                            }
                    }
                    if (b.goal < 0)
                        for (int gi : botGoalsOf(b.team))
                            if (botFits(g, gi, g.p[b.carry].isPin, b.rider >= 0 ? 2 : 1, bi)) b.goal = gi;
                    if (b.goal < 0) { botStartDump(g, b, bi); if (!b.dumping && b.dropT <= 0.0) { botRelease(g, b); b.backT = 0.5; b.backTurn = 0; } BotCmd cmd; cmd.skip = true; return cmd; }
                }
                homeGoal = b.goal;
                botApproachGoal(g, b, goals, tx, ty);  // lining-up point in front of it, then straight in
                // Raise the lift to just over the stack on the way in
                double dg = std::hypot(tx - b.x, ty - b.y);
                b.liftTarget = dg < sk.liftEarly * S ? liftDegFor(goalTopIn(g, b.goal) + 0.8) : 0.0;
            } else if (b.task == 2) {          // expert: turn a Toggle by backing the arms into it
                b.liftTarget = 0.0;
                const int k = b.taskK;
                if ((b.taskT += dt) > 10.0 * std::max(1.0, 76.6 / g_topIps) || g.toggle[k] == b.team) {
                    b.task = 0; b.taskK = -1; b.phase = 0;
                } else {
                    double fx, fy, sh;
                    toggleStage(k, fx, fy, sh);
                    const double herr = wrapPi(sh - b.h);
                    if (inLoadZone(b.x, b.y, b.h, BOT_HALF_IN * S, BOT_HALF_IN * S, b.team == C_RED ? C_BLUE : C_RED)) { b.task = 0; b.taskK = -1; b.phase = 0; b.escapeT = 0.0; }  // in the other alliance's Load Zone: give the Toggle up and get out (the penalty avoidance takes over)
                    else if (b.phase >= 1 && std::hypot(fx * S - b.x, (720.0 - fy * S) - b.y) > 16.0 * S) b.phase = 0;  // shoved or drifted away from the Toggle: line up again
                    if (b.task != 2) {}
                    else if (b.phase == 0) {        // drive to the spot in front of it
                        tx = fx * S; ty = 720.0 - fy * S;
                        if (std::hypot(tx - b.x, ty - b.y) < 4.0 * S) b.phase = 1;
                    } else {
                        direct = true;
                        if (b.phase == 1) {    // turn the back to the wall
                            turn = std::clamp(herr * 2.5, -1.0, 1.0);
                            if (std::fabs(herr) < 0.06) { b.phase = 2; b.tgHit = false; }
                        } else if (b.phase == 2) {  // back in until the arms roll it
                            fwd = -0.55;
                            turn = std::clamp(herr * 2.0, -0.3, 0.3);
                            if (b.tgHit) { b.phase = 3; b.phaseT = 0.0; }
                        } else {               // pull out 4 in so the arms re-arm, then again if needed
                            fwd = 0.45;
                            turn = std::clamp(herr * 2.0, -0.3, 0.3);
                            if ((b.phaseT += dt) > 0.4) { b.phase = 2; b.tgHit = false; }
                        }
                    }
                }
            } else if (b.task == 1) {          // expert: take a piece from one of its Loaders
                b.liftTarget = 0.0;
                const int li = b.taskK;
                auto& ls = g.loaderStack[li];
                const bool left = LOADERS[li].x < FIELD_IN / 2;
                const double ox = loaderOutX(li) * S, oy = 720.0 - LOADERS[li].y * S;
                const double sh = left ? M_PI : 0.0, nx = left ? 1.0 : -1.0;
                const double herr = wrapPi(sh - b.h);
                const double gx = b.x + c0 * grip, gy = b.y + s0 * grip;  // the claw's grip point
                const double dO = std::hypot(gx - ox, gy - oy);
                if ((b.taskT += dt) > 10.0 * std::max(1.0, 76.6 / g_topIps) || (ls.empty() && b.phase < 2)) {
                    b.task = 0; b.taskK = -1; b.phase = 0;
                } else if (b.phase == 0) {     // drive to the spot in front of it
                    tx = ox + nx * (BOT_GRIP_IN + 10.0) * S; ty = oy;
                    if (std::hypot(tx - b.x, ty - b.y) < 3.0 * S) b.phase = 1;
                } else {
                    direct = true;
                    if (b.phase == 1) {        // face the Loader
                        turn = std::clamp(herr * 2.5, -1.0, 1.0);
                        if (std::fabs(herr) < 0.05) { b.phase = 2; b.phaseT = 0.0; }
                    } else {                   // front slot around the Loader's foot, then wait for the piece
                        double ly = -(ox - b.x) * s0 + (oy - b.y) * c0;  // sideways error of the opening
                        turn = std::clamp(herr * 2.0 + ly / S * 0.08, -0.3, 0.3);
                        fwd = dO > 1.6 * S ? (dO > 6.0 * S ? sk.dock + 0.15 : sk.dock) : 0.0;
                        if (dO < LOADER_APPROACH_IN * S) g.botAtLoader[li] = true;
                        // Docked with the piece out: wait (up to 3 s) for the drive team's second piece so a
                        // pin and a cup come out together, then take them (the player's claw latches by itself)
                        const int team = b.team == C_RED ? 0 : 1;
                        const bool secondComing = ls.size() == 1 && (g.p[ls[0]].isPin ? g.loadCups[team] : g.loadPins[team]) > 0;
                        const bool ready = !ls.empty() && g.p[ls.front()].slide >= 0.95 && dO < 2.0 * S;
                        if (!pilot && ready && secondComing && (b.phaseT += dt) < 3.0) {
                            fwd = 0.0;
                        } else if (!pilot && ready) {
                            const int idx = ls.front();
                            ls.erase(ls.begin());
                            Piece& pc = g.p[idx];
                            pc.loader = -1; pc.slide = 0.0; pc.lz = pc.lzv = 0.0;
                            pc.carrier = bi; pc.lying = false; pc.vx = pc.vy = 0.0;
                            b.carry = idx;
                            ++b.loaded;
                            if (!ls.empty() && g.p[ls.front()].isPin != pc.isPin) {  // the other piece, nested on it
                                const int r = ls.front();
                                ls.erase(ls.begin());
                                Piece& rp = g.p[r];
                                rp.loader = -1; rp.slide = 0.0; rp.lz = rp.lzv = 0.0;
                                rp.carrier = bi; rp.lying = false; rp.vx = rp.vy = 0.0;
                                b.rider = r;
                                ++b.loaded;
                            }
                            b.task = 0; b.taskK = -1; b.phase = 0;
                            b.wait = sk.pause * 0.5;
                            b.backT = 0.6; b.backTurn = 0.0;
                            b.clawDeg += 120.0;
                        } else if (ls.empty() && (b.phaseT += dt) > 3.0) {
                            b.task = 0; b.taskK = -1; b.phase = 0; b.backT = 0.5; b.backTurn = 0.0;
                        }
                    }
                }
            } else {                           // go for a loose piece
                b.liftTarget = 0.0;
                if (b.target >= 0 && (b.targetT += dt) > 7.0 * std::max(1.0, 76.6 / g_topIps)) {  // chased it too long: leave it for a while
                    b.skip = b.target; b.skipT = 8.0; b.target = -1;
                }
                // Plan again when its piece is gone; with nothing to do, look again once a second
                bool idle = b.target < 0;
                bool need = idle ? (b.replanT += dt) > 1.0
                                 : (!botCanTake(g, bi, b.target) || b.goal < 0 || !botFits(g, b.goal, g.p[b.target].isPin));
                if (sk.smart) {
                    const bool eg = sk.endgame > 0.0 && g.matchT <= sk.endgame;
                    if (eg != b.endgame) { b.endgame = eg; need = true; }       // the endgame starts: new plan
                }
                if (need) {
                    if (sk.smart) botPlanSmart(g, bi, plx, ply, TOP * sk.stick * 0.6, sk);
                    else botPlan(g, bi, plx, ply);
                    b.targetT = b.replanT = 0.0;
                }
                if (b.task == 0 && b.target >= 0) {
                    botApproach(g, b, tx, ty);  // lining-up point, then straight in
                    homeGoal = b.goal;
                } else if (b.task == 0) {      // nothing to do: wait on its own side
                    tx = (b.team == C_RED ? 35.0 : 105.0) * S; ty = 720.0 - 70.0 * S;
                }
            }
        }
        // Penalty avoidance: never linger at an opposing Alliance Goal (<SG9>), in the other alliance's Load Zone or at its
        // Loaders (<SG13>). Once close, drive straight away from them (for at least half a second) before anything else.
        if (!direct) {
            const Col other = b.team == C_RED ? C_BLUE : C_RED;
            double ex = 0.0, ey = 0.0;
            bool risk = false;
            auto awayFrom = [&](double ox, double oy, double r) {
                const double d = std::hypot(b.x - ox, b.y - oy);
                if (d < r * S && d > 1e-6) { risk = true; ex += (b.x - ox) / d * (r * S - d); ey += (b.y - oy) / d * (r * S - d); }
            };
            for (int gi = 0; gi < 4; ++gi) if (!goalAllowed(b.team, gi)) awayFrom(goals[gi].x, goals[gi].y, BOT_HALF_IN + 2.8 + 8.0);
            for (const LoaderDef& ld : LOADERS) if (ld.c == other) awayFrom(ld.x * S, 720.0 - ld.y * S, BOT_HALF_IN + 6.0);
            // an opposing robot in (or next to) its own Load Zone is protected: touching it is a violation, so keep two robot widths off it
            auto protectedFoe = [&](double x, double y, double h, Col t) {
                if (t != b.team && inLoadZone(x, y, h, (BOT_HALF_IN + 4.0) * S, (BOT_HALF_IN + 4.0) * S, t)) awayFrom(x, y, 2.0 * BOT_HALF_IN + 8.0);
            };
            protectedFoe(plx, ply, 0.0, C_RED);
            for (int j = 0; j < BOT_N; ++j) if (j != bi && g.bots[j].on) protectedFoe(g.bots[j].x, g.bots[j].y, g.bots[j].h, g.bots[j].team);
            if (inLoadZone(b.x, b.y, b.h, BOT_HALF_IN * S, BOT_HALF_IN * S, other)) { risk = true; ex += 360.0 - b.x; ey += 360.0 - b.y; }
            if (risk) {
                const double el = std::max(1e-6, std::hypot(ex, ey));
                b.escX = std::clamp(b.x + ex / el * 40.0 * S, 12.0 * S, 720.0 - 12.0 * S);
                b.escY = std::clamp(b.y + ey / el * 40.0 * S, 12.0 * S, 720.0 - 12.0 * S);
                b.escapeT = 1.2;
            }
            if (b.escapeT > 0.0) { b.escapeT -= dt; tx = b.escX; ty = b.escY; homeGoal = -1; defending = false; }
        }
        if (!direct) {
            // Detour: if the straight line to the target passes a goal or Loader, head first for a
            // point beside it, on whichever side stays clear of the walls and is shorter
            const double realTx = tx, realTy = ty;
            {
                const double ax = b.x, ay = b.y, sx = tx - ax, sy = ty - ay, len = std::hypot(sx, sy);
                if (len > 1e-6) {
                    double bestT = 1e18, hx = 0, hy = 0, hr = 0;
                    auto check = [&](double ox, double oy, double r) {
                        double t = ((ox - ax) * sx + (oy - ay) * sy) / (len * len);
                        if (t <= 0.0 || t >= 1.0) return;
                        double qx = ax + sx * t, qy = ay + sy * t;
                        if (std::hypot(ox - qx, oy - qy) < (BOT_HALF_IN + r + 2.0) * S && t < bestT) { bestT = t; hx = ox; hy = oy; hr = r; }
                    };
                    for (int gi = 0; gi < 9; ++gi)
                        if (gi != homeGoal && std::hypot(goals[gi].x - tx, goals[gi].y - ty) > 6.0 * S) check(goals[gi].x, goals[gi].y, 2.8);
                    for (const LoaderDef& ld : LOADERS) check(ld.x * S, 720.0 - ld.y * S, 2.1);
                    for (const LoaderDef& ld : LOADERS) if (ld.c != b.team) check((ld.x < FIELD_IN / 2 ? LOADZONE_X / 2 : FIELD_IN - LOADZONE_X / 2) * S, 720.0 - ld.y * S, 11.0);  // go round the other alliance's Load Zones, not through them
                    if (bestT < 1e17) {
                        const double nx = -sy / len, ny = sx / len, off = (BOT_HALF_IN + hr + 5.0) * S, lo = 11.0 * S, hi = 720.0 - 11.0 * S;
                        double best = 1e18;
                        for (double sd : {1.0, -1.0}) {
                            double cx = hx + nx * off * sd, cy = hy + ny * off * sd;
                            if (cx < lo || cx > hi || cy < lo || cy > hi) continue;  // that side is against a wall
                            double l = std::hypot(cx - ax, cy - ay) + std::hypot(tx - cx, ty - cy);
                            if (l < best) { best = l; tx = cx; ty = cy; }
                        }
                    }
                }
            }
            (void)realTx; (void)realTy;
            // Steer: straight at the target, bent away from goals, Loaders and other robots nearby
            double dx = tx - b.x, dy = ty - b.y, D = std::hypot(dx, dy);
            double ux = D > 1e-6 ? dx / D : 0.0, uy = D > 1e-6 ? dy / D : 0.0;
            // Each obstacle pushes the robot away and also round its side toward the target, so one
            // sitting straight between them doesn't cancel the pull and leave the robot stopped
            const double tux = ux, tuy = uy;
            auto repel = [&](double ox, double oy, double R, double k) {
                double ex = b.x - ox, ey = b.y - oy, d = std::hypot(ex, ey);
                if (d >= R || d < 1e-6) return;
                double rx = ex / d, ry = ey / d, px = -ry, py = rx;
                // Go round on the side toward the target, leaning to the open middle of the field
                // (an 18 in robot rarely fits between a goal and the wall)
                double cx = 360.0 - b.x, cy = 360.0 - b.y, cl = std::max(1e-6, std::hypot(cx, cy));
                double side = px * tux + py * tuy + 0.6 * (px * cx + py * cy) / cl;
                if (std::fabs(side) < 0.05) side = (bi & 1) ? 1.0 : -1.0;  // dead ahead: pick a side
                if (side < 0) { px = -px; py = -py; }
                double m = (R - d) / R * k;
                ux += (rx * 0.7 + px * 0.9) * m;
                uy += (ry * 0.7 + py * 0.9) * m;
            };
            for (int gi = 0; gi < 9; ++gi)
                if (gi != homeGoal && std::hypot(goals[gi].x - tx, goals[gi].y - ty) > 6.0 * S)
                    repel(goals[gi].x, goals[gi].y, (2.8 + BOT_HALF_IN + (!goalAllowed(b.team, gi) ? 12.0 : 6.0)) * S, !goalAllowed(b.team, gi) ? 2.6 : 1.6);
            for (const LoaderDef& ld : LOADERS) repel(ld.x * S, 720.0 - ld.y * S, (2.1 + BOT_HALF_IN + 3.0) * S, 1.3);
            // Object detection: loose pieces that are not its target are steered round instead of driven through (a shoved piece ends up in the claw's way or out of reach)
            for (int pi2 = 0; pi2 < (int)g.p.size(); ++pi2) {
                const Piece& opc = g.p[pi2];
                if (!loose(opc) || pi2 == b.target || std::hypot(opc.x - b.x, opc.y - b.y) > 24.0 * S) continue;
                repel(opc.x, opc.y, 11.0 * S, 0.5);
            }
            for (const LoaderDef& ld : LOADERS)  // the other alliance's Load Zones: pass through at most (<SG13>)
                if (ld.c != b.team) repel((ld.x < FIELD_IN / 2 ? LOADZONE_X / 2 : FIELD_IN - LOADZONE_X / 2) * S, 720.0 - ld.y * S, (TILE_IN / 2 + BOT_HALF_IN + 8.0) * S, 2.5);
            if (!defending) repel(plx, ply, (b.team == C_BLUE ? 15.0 : 27.0) * S, b.team == C_BLUE ? 0.4 : 1.0);  // opponents barely give way: more clashes
            for (int j = 0; j < BOT_N; ++j)
                if (j != bi && g.bots[j].on) {
                    const bool foe = g.bots[j].team != b.team;
                    if (foe && defending) continue;
                    repel(g.bots[j].x, g.bots[j].y, (foe ? 15.0 : 26.0) * S, foe ? 0.4 : 1.0);
                }
            // Walls: a robot turning close to one scrubs a corner on it (the corners sweep 12.7 in from the centre) and loses speed, so the
            // steering keeps its centre about 16 in off the walls, unless the target itself is at that wall
            if (!defending) {
                const double R = 16.0 * S;
                auto wallPush = [&](double d, double nx, double ny, double tgtD) {
                    if (d >= R || tgtD < 14.0 * S) return;
                    const double m = (R - d) / R * 1.5;
                    ux += nx * m; uy += ny * m;
                };
                wallPush(b.x, 1.0, 0.0, tx); wallPush(720.0 - b.x, -1.0, 0.0, 720.0 - tx);
                wallPush(b.y, 0.0, 1.0, ty); wallPush(720.0 - b.y, 0.0, -1.0, 720.0 - ty);
            }
            // The target sits at the claw, not the robot's centre
            double err = wrapPi(std::atan2(uy, ux) - b.h);
            turn = std::clamp(err * 2.0, -1.0, 1.0);
            fwd = std::fabs(err) > 1.2 ? 0.0 : std::cos(err) * std::clamp((D - grip) / (24.0 * S), sk.approach, 1.0);
            if (!defending && D < 22.0 * S && (b.carry >= 0 || b.target >= 0))
                fwd *= std::pow(std::max(0.0, std::cos(err)), sk.lineUp);  // line up squarely first
            if (defending && D < 3.0 * S) fwd = 0.0;
            // Carrying to a goal: hold short of it until the lift is up over the stack
            if (!defending && b.carry >= 0 && D < (BOT_HALF_IN + 9.0) * S && b.liftDeg < b.liftTarget - 20.0) fwd = 0.0;
            // Turning with the chassis against a wall or goal scrubs speed: when it has a big turn to make with something close, it creeps away from it while turning
            if (std::fabs(err) > 0.5 && b.backT <= 0.0) {
                double ax = 0.0, ay = 0.0;
                const double wr = (BOT_HALF_IN + 4.0) * S;
                if (b.x < wr) ax += wr - b.x;
                if (720.0 - b.x < wr) ax -= wr - (720.0 - b.x);
                if (b.y < wr) ay += wr - b.y;
                if (720.0 - b.y < wr) ay -= wr - (720.0 - b.y);
                for (int gi = 0; gi < 9; ++gi) {
                    if (gi == homeGoal) continue;
                    const double dd = std::hypot(b.x - goals[gi].x, b.y - goals[gi].y), rr = (BOT_HALF_IN + 2.8 + 4.0) * S;
                    if (dd < rr && dd > 1e-6) { ax += (b.x - goals[gi].x) / dd * (rr - dd); ay += (b.y - goals[gi].y) / dd * (rr - dd); }
                }
                const double al = std::hypot(ax, ay);
                if (al > 1.0 * S) {
                    const double dotp = (ax * c0 + ay * s0) / al;
                    if (std::fabs(dotp) > 0.25) fwd = std::clamp(fwd + 0.4 * (dotp > 0 ? 1.0 : -1.0), -0.5, 0.6);
                }
            }
        }
    }

    BotCmd cmd;
    cmd.fwd = fwd; cmd.turn = turn; cmd.defending = defending;
    return cmd;
}

// The referee: a Minor Violation (a warning); the third in a match makes it a Major Violation and the
// alliance is Disqualified (<SG13>, and as the other rules allow)
inline void callViolation(Game& g, Col team, const std::string& who, const std::string& what) {
    const int a = team == C_RED ? 0 : 1;
    if (++g.minors[a] >= 3) g.dq[a] = true;
    if (std::getenv("VEXSIM_TRACE")) std::fprintf(stderr, "VIOLATION t %.1f %s %s %s\n", g_matchLen - g.matchT, team == C_RED ? "red" : "blue", who.c_str(), what.c_str());
    g.violMsg = std::string(team == C_RED ? "RED " : "BLUE ") + who + ": " +
                (g.dq[a] ? "MAJOR VIOLATION " + what + " - DISQUALIFIED" : "MINOR VIOLATION " + what + " (WARNING " + std::to_string(g.minors[a]) + ")");
    g.violMsgT = 4.0;
    if (std::getenv("VEXSIM_VIOLATIONS")) if (FILE* vf = std::fopen("vexsim_violations.log", "a")) { std::fprintf(vf, "t %.1f %s %s %s\n", g_matchLen - g.matchT, team == C_RED ? "red" : "blue", who.c_str(), what.c_str()); std::fclose(vf); }
}
// Is a robot (centre and heading in sim units, half length / width in px) at least partly in a Load
// Zone of this colour? Red's are in the left corners, blue's in the right
inline bool inLoadZone(double x, double y, double h, double hl, double hw2, Col zone) {
    const double zx0 = zone == C_RED ? 0.0 : FIELD_IN - LOADZONE_X - 0.7, zx1 = zone == C_RED ? LOADZONE_X + 0.7 : FIELD_IN;
    const double ext = (std::fabs(std::cos(h)) * hl + std::fabs(std::sin(h)) * hw2) / PX_IN;
    const double eyt = (std::fabs(std::sin(h)) * hl + std::fabs(std::cos(h)) * hw2) / PX_IN;
    const double fx = x / PX_IN, fy = (720.0 - y) / PX_IN;
    return fx + ext > zx0 && fx - ext < zx1 && (fy - eyt < TILE_IN + 0.7 || fy + eyt > FIELD_IN - TILE_IN - 0.7);
}

// A robot holding something no goal can take (or that it has carried too long without placing) drops it and goes for a new load
inline void botDropStuck(Game& g, Bot& b, int bi, double dt) {
    if (b.dropT > 0.0) { b.dropT -= dt; return; }  // (counts down whether or not the piece is still in the claw)
    if (b.carry < 0) { b.carryT = 0.0; b.noGoalT = 0.0; b.dumping = false; return; }
    if (b.dumping) return;  // on its way to the dump spot
    b.carryT += dt;
    bool any = false;
    const int need = b.rider >= 0 ? 2 : 1;
    for (int gi = 0; gi < 9 && !any; ++gi)
        if (goalAllowed(b.team, gi) && botFits(g, gi, g.p[b.carry].isPin, need, bi)) any = true;
    b.noGoalT = any ? 0.0 : b.noGoalT + dt;
    // Holding a piece but going nowhere (blocked, or no way to reach a goal): drop it after 3 s
    if (std::hypot(b.x - b.dsX, b.y - b.dsY) > 3.0 * PX_IN) { b.dsX = b.x; b.dsY = b.y; b.dsT = 0.0; }
    else if (b.wait <= 0.0 && b.backT <= 0.0 && !(b.task == 1 && b.phase >= 2)) b.dsT += dt;
    if (b.noGoalT > 0.4 || b.carryT > 9.0 * std::max(1.0, 76.6 / g_topIps) || b.dsT > 3.0) {
        b.dsT = 0.0;
        botStartDump(g, b, bi);  // out of its way first; no spot: here
        if (!b.dumping) {
            if (bi == BOT_N) b.dropT = 1.5;  // its piece is in the real claw: the claw runs out (pilotUpdate)
            else botRelease(g, b);
        }
        b.carryT = b.noGoalT = 0.0; b.backT = b.dumping ? 0.0 : 0.6; b.backTurn = 0.0;
        b.cyc = 0; b.cycT = 0.0; b.cycOff = 0.0; b.task = 0; b.target = -1; b.goal = -1;
    }
}

// One step for the other robots. The computer works the sticks; everything after that is this
// robot: the robot code's split arcade mixing (TURN_SCALE 0.4, shares pulled equal past full
// power), the drive's top speed, motor lag and wheel grip, the lift at the code's 70 percent,
// pieces taken and placed at the claw's grip point inside the front opening, and the passive
// toggle arms on the back of the lift.
inline void botsUpdate(SimState& s, const RobotConfig& cfg, const std::array<GoalObstacle, 9>& goals, double dt) {
    Game& g = game();
    const double S = PX_IN, hb = BOT_HALF_IN * S, grip = BOT_GRIP_IN * S;
    double TOP = g_topIps * S; const double TRACK = 13.0 * S;      // 450 rpm on 3.25 in wheels; 13 in track
    const double TAU = 0.15, AMAX = 0.8 * G_IN_S2 * S;  // motor lag and wheel grip, as in Physics::step
    double plx, ply, plh;
    {
        std::lock_guard<std::mutex> lock(s.pose_mutex);
        plx = s.pose.x; ply = s.pose.y; plh = s.pose.heading;
    }
    const bool playerHolding = g.held >= 0;
    g.botAtLoader = {};
    if (dt > 0) botsMatchLoads(g, dt, botSkill(g.bots[0].level).loaders || botSkill(g.bots[3].level).loaders || g.set.pilot >= 2,
                   botSkill(g.bots[1].level).loaders || botSkill(g.bots[2].level).loaders,
                   std::min(botSkill(g.bots[0].level).reload, botSkill(g.bots[3].level).reload),
                   std::min(botSkill(g.bots[1].level).reload, botSkill(g.bots[2].level).reload));

    for (int bi = 0; bi < BOT_N && dt > 0; ++bi) {
        Bot& b = g.bots[bi];
        if (!b.on) continue;
        const BotSkill sk = botSkill(b.level);
        g_topIps = b.topIps; TOP = g_topIps * S;  // this robot's own drivetrain speed
        if (g.bracketCur < 0) b.omni = g.set.omniBot[bi < 3 ? bi : 3];
        setLiftType(b.lift);                       // and its own lift
        // SCORE ON: a robot turns the pin it carries so the half it scores shows: yellow, or its own colour
        for (int pi : {b.carry, b.rider}) {
            if (pi < 0 || !g.p[pi].isPin) continue;
            Piece& pcs = g.p[pi];
            const Col want = botYel(b) ? C_YEL : b.team;
            if (pcs.top != want && pcs.bot == want) std::swap(pcs.top, pcs.bot);
        }
        { const int nf = botModeNow(g, bi, b); if (nf != b.focus) { b.focus = nf; b.cyc = 0; b.cycT = 0.0; b.task = 0; b.target = -1; } }
        botDropStuck(g, b, bi, dt);
        if (b.skipT > 0 && (b.skipT -= dt) <= 0) b.skip = -1;
        const BotCmd cmd = botDecide(g, bi, b, sk, goals, dt, plx, ply, playerHolding, false);
        if (cmd.skip) continue;
        const double liftIn = dr4bLiftIn(b.liftDeg);
        const double c0 = std::cos(b.h), s0 = std::sin(b.h);
        double fwd = cmd.fwd, turn = cmd.turn;
        const bool defending = cmd.defending;

        // Claw: a piece at the grip point is taken (the ratchet latches onto it), and a held piece
        // over a goal it fits on, with the lift high enough, is let go onto the stack
        auto local = [&](double px, double py, double& lx, double& ly) {
            lx = (px - b.x) * c0 + (py - b.y) * s0;
            ly = -(px - b.x) * s0 + (py - b.y) * c0;
        };
        if (b.carry >= 0 && b.goal >= 0 && !defending) {
            double lx, ly;
            local(goals[b.goal].x, goals[b.goal].y, lx, ly);
            if (std::fabs(lx - grip) < 1.0 * S && std::fabs(ly) < 1.3 * S && liftIn >= goalTopIn(g, b.goal) - 0.4 &&
                (std::rand() / (double)RAND_MAX) >= sk.placeOK) {  // fumbled: the piece drops to the floor
                botRelease(g, b);
                b.backT = sk.backOff + 0.4; b.backTurn = 0.0;
                b.wait = sk.pause + sk.idle * (0.5 + (std::rand() / (double)RAND_MAX));
            } else if (std::fabs(lx - grip) < 1.0 * S && std::fabs(ly) < 1.3 * S && liftIn >= goalTopIn(g, b.goal) - 0.4) {
                Piece& pc = g.p[b.carry];
                auto& st = g.stack[b.goal];
                pc.placedBy = bi;
                pc.carrier = -1;
                pc.goal = b.goal;
                pc.lying = false;
                pc.x = goals[b.goal].x; pc.y = goals[b.goal].y;
                pc.fz = std::max(0.0, liftIn - stackBottomIn(b.goal, (int)st.size()));
                pc.fzv = 0.0;
                st.push_back(b.carry);
                ++b.placed;
                if (b.rider >= 0) {  // the nested piece goes on with it
                    Piece& rp = g.p[b.rider];
                    rp.placedBy = bi;
                    rp.carrier = -1;
                    rp.goal = b.goal;
                    rp.lying = false;
                    rp.x = pc.x; rp.y = pc.y;
                    rp.fz = std::max(0.0, liftIn + HALF_IN - stackBottomIn(b.goal, (int)st.size()));
                    rp.fzv = 0.0;
                    st.push_back(b.rider);
                    b.rider = -1;
                    ++b.placed;
                }
                b.carry = b.goal = -1;
                if (b.focus == FOCUS_SCORING) { b.wait = 0.0; b.backT = 0.0; }  // the cycle backs out along its own line
                else {
                    b.wait = sk.pause * (0.6 + 0.8 * (std::rand() / (double)RAND_MAX)) + sk.idle * (0.5 + (std::rand() / (double)RAND_MAX));
                    b.backT = sk.backOff; b.backTurn = 0.0;
                }
                b.clawDeg -= 120.0;
            }
        } else if (b.carry < 0 && b.task == 0 && liftIn < 1.2 && !defending) {
            for (int i = 0; i < (int)g.p.size(); ++i) {
                const Piece& pc = g.p[i];
                if (!loose(pc) || pc.fz > 0.5) continue;
                if (sk.smart && i != b.target) continue;  // expert only takes what it planned for
                double lx, ly;
                local(pc.x, pc.y, lx, ly);
                if (std::fabs(lx - grip) > 1.2 * S || std::fabs(ly) > 1.3 * S) continue;
                if (!botCanTake(g, bi, i)) continue;
                int gl = -1;  // its target, or any piece that fits on one of its goals
                if (i == b.target) gl = b.goal;
                else for (int gi : botGoalsOf(b.team)) if (botFits(g, gi, pc.isPin)) gl = gi;
                if (gl < 0) continue;
                if ((std::rand() / (double)RAND_MAX) >= sk.grabOK) {  // missed it: back off and leave it a moment
                    b.backT = 0.6; b.backTurn = 0.0;
                    b.wait = sk.pause;
                    b.skip = i; b.skipT = 3.0; b.target = -1;
                    break;
                }
                const int pinIn = botPinIn(g, i);
                Piece& tp = g.p[i];
                tp.carrier = bi;
                tp.lying = false;
                tp.tilt = tp.tiltV = 0.0;
                tp.inCup = -1;
                tp.vx = tp.vy = tp.spin = 0.0;
                b.carry = i;
                if (pinIn >= 0) {  // the pin standing in the cup comes too, nested on it
                    Piece& rp = g.p[pinIn];
                    rp.carrier = bi; rp.inCup = -1; rp.lying = false; rp.tilt = rp.tiltV = 0.0; rp.vx = rp.vy = 0.0;
                    b.rider = pinIn;
                }
                b.goal = gl;
                b.target = -1;
                b.wait = sk.pause * 0.5;
                b.clawDeg += 120.0;
                fwd = 0.0;
                break;
            }
        }

        // The robot code's split arcade, then the drive: each side's wheels follow their command
        // with the motors' lag and no faster than the wheels can grip
        double f = driveNorm(std::clamp(fwd, -1.0, 1.0), b.topIps) * 100.0 * sk.stick;
        double t = driveNorm(std::clamp(turn, -1.0, 1.0), b.topIps) * 100.0 * sk.stick * BOT_TURN_SCALE;
        double sum = std::fabs(f) + std::fabs(t);
        if (sum > 100.0) {
            double share = std::fabs(f) / sum;
            share += (0.5 - share) * std::min(1.0, (sum - 100.0) / 100.0);
            f = std::copysign(100.0 * share, f);
            t = std::copysign(100.0 * (1.0 - share), t);
        }
        const double tL = (f + t) / 100.0 * TOP, tR = (f - t) / 100.0 * TOP;
        {
            const double brpm = b.topIps * 60.0 / (M_PI * 3.25);  // its drive torque follows its rpm (4 motors)
            const double dvL = (tL - b.vL) * std::min(1.0, dt / TAU), dvR = (tR - b.vR) * std::min(1.0, dt / TAU);
            const double aL = std::min(AMAX, driveAccelLimit(brpm, b.vL, TOP, dvL)), aR = std::min(AMAX, driveAccelLimit(brpm, b.vR, TOP, dvR));
            b.vL += std::clamp(dvL, -aL * dt, aL * dt);
            b.vR += std::clamp(dvR, -aR * dt, aR * dt);
        }
        const double v = (b.vL + b.vR) / 2, w = (b.vL - b.vR) / TRACK;
        b.h = wrapPi(b.h + w * dt);
        {   // the wheels set the speed along the heading; sideways speed (left over from a turn or a push)
            // bleeds off only as fast as the wheels grip sideways: traction middle wheels 0.6 g, all omni 0.35 g
            const bool omni = b.omni;
            const double hx = std::cos(b.h), hy = std::sin(b.h), nx = -hy, ny = hx;
            double side = b.vx * nx + b.vy * ny;
            const double bleed = (omni ? 0.35 : 0.6) * G_IN_S2 * S * dt;
            side = side > 0 ? std::max(0.0, side - bleed) : std::min(0.0, side + bleed);
            b.vx = hx * v + nx * side;
            b.vy = hy * v + ny * side;
        }
        b.x += b.vx * dt;
        b.y += b.vy * dt;
        b.wheelRotL += b.vL / (1.625 * S) * dt;
        b.wheelRotR += b.vR / (1.625 * S) * dt;
        // Lift: runs at the code's 70 percent toward its target, easing in at the end
        {
            double dl = b.liftTarget - b.liftDeg;
            double step = BOT_LIFT_DPS * g_liftSpeed * dt * std::clamp(std::fabs(dl) / 60.0, 0.2, 1.0);
            b.liftDeg = std::clamp(b.liftDeg + std::clamp(dl, -step, step), 0.0, LIFT_MAX_DEG);
        }

        botWatchdog(b, dt, defending);
        // Stuck (pinned against something while it has a job): back out, turn, and leave that piece for a while
        if ((b.stuckT += dt) > 1.5) {
            const bool driving = b.task == 0 ? (b.target >= 0 || b.carry >= 0) : b.phase == 0;  // not while turning on the spot or docked
            if (std::hypot(b.x - b.chkX, b.y - b.chkY) < 3.0 * S && b.wait <= 0 && b.backT <= 0 && !defending &&
                driving && std::fabs(b.liftTarget - b.liftDeg) < 20.0) {
                b.backT = 0.7;
                b.backTurn = (std::rand() & 1) ? 0.8 : -0.8;
                if (b.target >= 0) { b.skip = b.target; b.skipT = 6.0; b.target = -1; }
            }
            b.stuckT = 0.0; b.chkX = b.x; b.chkY = b.y;
        }
    }

    // Pinning (5 s): a robot driving into an opposing robot that is held against a wall or goal and cannot get away is called after 5 s
    if (dt > 0)
        for (Bot& q : g.bots) {
            if (!q.on) continue;
            if (q.pinNow) q.pinT += dt; else q.pinT = std::max(0.0, q.pinT - 2.0 * dt);
            q.pinCool = std::max(0.0, q.pinCool - dt);
            if (q.pinT > 5.0 && q.pinCool <= 0.0) { callViolation(g, q.team, q.num, "<G> PINNING"); q.pinCool = 5.0; q.pinT = 0.0; }
            q.pinNow = false;
        }
    // Collisions. The robots are the same machine (the player's has a little more torque)
    for (int bi = 0; bi < BOT_N; ++bi) {
        Bot& b = g.bots[bi];
        if (!b.on) continue;
        setLiftType(b.lift);
        double mx, my;
        if (b.hitCool > 0) b.hitCool -= dt;
        b.protT = std::max(0.0, b.protT - dt);
        g.playerProtT = std::max(0.0, g.playerProtT - dt / BOT_N);
        for (int j = bi + 1; j < BOT_N; ++j) {
            Bot& o = g.bots[j];
            if (o.on && obbPush(b.x, b.y, b.h, hb, hb, o.x, o.y, o.h, hb, hb, mx, my)) {
                // <SG13a>: a robot in its own Load Zone is protected; the opponent touching it is called
                if (b.team != o.team && dt > 0) {
                    if (inLoadZone(o.x, o.y, o.h, hb, hb, o.team) && b.hitCool <= 0) { if ((b.protT += 2.0 * dt) > 3.0) { callViolation(g, b.team, b.num, "<SG13> PROTECTED ROBOT"); b.hitCool = 3.0; b.protT = 0.0; } }
                    else if (inLoadZone(b.x, b.y, b.h, hb, hb, b.team) && o.hitCool <= 0) { if ((o.protT += 2.0 * dt) > 3.0) { callViolation(g, o.team, o.num, "<SG13> PROTECTED ROBOT"); o.hitCool = 3.0; o.protT = 0.0; } }
                }
                if (b.team != o.team && dt > 0) {  // pinning: a is driving into v, which is hardly moving and has a wall or goal right behind it
                    const double ml2 = std::max(1e-9, std::hypot(mx, my)), nx2 = mx / ml2, ny2 = my / ml2;  // from o to b
                    auto pins = [&](const Bot& a, const Bot& v, double ex, double ey) {  // (ex, ey): from a to v
                        const double closing = std::max(a.vx * ex + a.vy * ey, (a.vL + a.vR) / 2.0 * (std::cos(a.h) * ex + std::sin(a.h) * ey));
                        if (closing < 2.0 * S || std::hypot(v.vx, v.vy) > 3.0 * S) return false;
                        const double bx3 = v.x + ex * (hb + 5.0 * S), by3 = v.y + ey * (hb + 5.0 * S);
                        if (bx3 < 0.0 || bx3 > 720.0 || by3 < 0.0 || by3 > 720.0) return true;
                        for (int gi = 0; gi < 9; ++gi) if (std::hypot(bx3 - goals[gi].x, by3 - goals[gi].y) < goals[gi].r + 3.0 * S) return true;
                        return false;
                    };
                    if (pins(b, o, -nx2, -ny2)) b.pinNow = true;
                    if (pins(o, b, nx2, ny2)) o.pinNow = true;
                }
                {   // the robot driving harder into the other wins the push: the share each gives way follows how fast the other is shoving
                    const double ml = std::max(1e-9, std::hypot(mx, my)), nx = mx / ml, ny = my / ml;  // n points from o to b
                    // a shove is won by whoever pushes harder: speed into the other, and drive torque (600 / rpm: a slower drive has more)
                    const double tqB = 600.0 / (b.topIps * 60.0 / (M_PI * 3.25)), tqO = 600.0 / (o.topIps * 60.0 / (M_PI * 3.25));
                    const double pb = (std::max(0.0, -(b.vx * nx + b.vy * ny)) + 8.0 * S) * tqB;  // b closing on o
                    const double po = (std::max(0.0, o.vx * nx + o.vy * ny) + 8.0 * S) * tqO;     // o closing on b
                    const double gb = po / (pb + po);                                        // the share b is pushed back
                    b.x += mx * gb; b.y += my * gb;
                    o.x -= mx * (1.0 - gb); o.y -= my * (1.0 - gb);
                    if (dt > 0) {  // the shove keeps the loser sliding, and the wheels of the one pushed back fight the other
                        b.vx += mx * gb / dt * 0.25; b.vy += my * gb / dt * 0.25;
                        o.vx -= mx * (1.0 - gb) / dt * 0.25; o.vy -= my * (1.0 - gb) / dt * 0.25;
                        if (gb > 0.5) { b.vL *= 0.85; b.vR *= 0.85; } else { o.vL *= 0.85; o.vR *= 0.85; }
                    }
                }
            }
        }
        if (obbPush(b.x, b.y, b.h, hb, hb, plx, ply, plh, cfg.robot_half_w, cfg.robot_half_h, mx, my)) {
            if (b.team == C_BLUE && dt > 0) {  // <SG13a> between a blue robot and the player's (red)
                if (inLoadZone(plx, ply, plh, cfg.robot_half_w, cfg.robot_half_h, C_RED) && b.hitCool <= 0) { if ((b.protT += 2.0 * dt) > 3.0) { callViolation(g, C_BLUE, b.num, "<SG13> PROTECTED ROBOT"); b.hitCool = 3.0; b.protT = 0.0; } }
                else if (inLoadZone(b.x, b.y, b.h, hb, hb, C_BLUE) && g.playerHitCool <= 0) { if ((g.playerProtT += 2.0 * dt) > 3.0) { callViolation(g, C_RED, "41658F", "<SG13> PROTECTED ROBOT"); g.playerHitCool = 3.0; g.playerProtT = 0.0; } }
            }
            // The player's robot pushes a little harder; an all-omni robot gives way more when shoved sideways
            const bool omni = b.omni;
            const double sideways = std::fabs(-std::sin(b.h) * mx + std::cos(b.h) * my) / std::max(1e-9, std::hypot(mx, my));
            const double give = 1.0 / (1.0 + PLAYER_TORQUE * (omni ? 1.0 + 0.8 * sideways : 1.0));
            b.x += mx * (1.0 - give); b.y += my * (1.0 - give);
            plx -= mx * give; ply -= my * give;
            if (dt > 0) { b.vx += mx * (1.0 - give) / dt * 0.3; b.vy += my * (1.0 - give) / dt * 0.3; }  // the shove keeps it sliding
            std::lock_guard<std::mutex> lock(s.pose_mutex);
            s.pose.x = plx; s.pose.y = ply;
        }
        const double c = std::cos(b.h), sn = std::sin(b.h);
        const double liftIn = dr4bLiftIn(b.liftDeg);
        auto slow = [&]() { b.vL *= 0.3; b.vR *= 0.3; };
        auto toLocal = [&](double ox, double oy, double& lx, double& ly) {
            double dx = ox - b.x, dy = oy - b.y;
            lx = dx * c + dy * sn; ly = -dx * sn + dy * c;
        };
        auto pushOff = [&](double ox, double oy, double r) {
            double lx, ly;
            toLocal(ox, oy, lx, ly);
            double qx = std::clamp(lx, -hb, hb), qy = std::clamp(ly, -hb, hb);
            double ex = lx - qx, ey = ly - qy, d = std::hypot(ex, ey);
            if (d >= r) return;
            double nlx, nly;
            if (d > 1e-9) { nlx = ex / d; nly = ey / d; }
            else { double l = std::hypot(lx, ly); nlx = l > 1e-9 ? lx / l : 1.0; nly = l > 1e-9 ? ly / l : 0.0; }
            double push = r - d;
            b.x -= (nlx * c - nly * sn) * push;
            b.y -= (nlx * sn + nly * c) * push;
            if ((b.vL + b.vR) * nlx > 0) slow();  // driving into it
        };
        // Goals: one lined up with the front opening comes in until it reaches the claw's grip point
        // (the alignment guide centres it), if the claw is high enough to pass over it; otherwise the body stops it
        bool onTheirGoal = false;
        for (int gi = 0; gi < 4; ++gi)  // <SG9>: opposing Alliance Goals may not be touched
            if ((gi < 2) != (b.team == C_RED)) {
                double lx, ly;
                toLocal(goals[gi].x, goals[gi].y, lx, ly);
                const double ex = lx - std::clamp(lx, -hb, hb), ey = ly - std::clamp(ly, -hb, hb);
                if (std::hypot(ex, ey) < goals[gi].r + 0.3 * S) onTheirGoal = true;
            }
        if (onTheirGoal && dt > 0) {
            if ((b.goalT += dt) > 3.0) { callViolation(g, b.team, b.num, "<SG9> OPPOSING GOAL"); b.goalT = -4.0; }
        } else if (b.goalT > 0) b.goalT = 0.0;
        for (int gi = 0; gi < 9; ++gi) {
            const auto& go = goals[gi];
            double lx, ly;
            toLocal(go.x, go.y, lx, ly);
            const bool clear = b.carry >= 0 && gi == b.goal && liftIn >= goalTopIn(g, gi) - 0.4;  // only the goal it is placing on
            if (clear && std::fabs(ly) < OPEN_HALF_IN * S && lx > hb - OPEN_DEPTH_IN * S && lx < hb + go.r) {
                if (lx < grip) {
                    b.x -= c * (grip - lx); b.y -= sn * (grip - lx);
                    if (b.vL + b.vR > 0) slow();
                }
                double shift = ly * 0.25;  // the angled guide slides the robot sideways onto it
                b.x += -sn * shift; b.y += c * shift;
                continue;
            }
            pushOff(go.x, go.y, go.r);
        }
        // Loaders: the front slot goes around a Loader's foot until its opening is at the claw
        for (int li = 0; li < 4; ++li) {
            const double ox = LOADERS[li].x * S, oy = 720.0 - LOADERS[li].y * S;
            double lx, ly;
            toLocal(ox, oy, lx, ly);
            if (std::fabs(ly) < (OPEN_HALF_IN - 2.1) * S && lx > hb - OPEN_DEPTH_IN * S && lx < hb + 2.1 * S) {
                const double stop = grip + 2.2 * S;  // the foot opening (2.2 in out from the Loader) sits at the grip point
                if (lx < stop) {
                    b.x -= c * (stop - lx); b.y -= sn * (stop - lx);
                    if (b.vL + b.vR > 0) slow();
                }
                continue;
            }
            pushOff(ox, oy, 2.1 * S);
        }
        // Walls
        double ext = (std::fabs(c) + std::fabs(sn)) * hb;
        double nx = std::clamp(b.x, ext, 720.0 - ext), ny = std::clamp(b.y, ext, 720.0 - ext);
        if (nx != b.x || ny != b.y) {
            if ((nx - b.x) * c * (b.vL + b.vR) + (ny - b.y) * sn * (b.vL + b.vR) < 0) slow();
            b.x = nx; b.y = ny;
        }
        // Toggle arms on the back of the lift: at Toggle height a Toggle stops them, and backing into
        // one at 6 in/s or more rolls it one face (once per ram; they re-arm after backing off 1.5 in)
        {
            double padX, padZ;
            ramPadPose(dr4bAngleRad(b.liftDeg), BOT_HALF_IN, padX, padZ);
            b.touchTog = {};
            if (padZ >= RAM_Z_LO && padZ <= RAM_Z_HI) {
                const double F = FIELD_IN, contact = TOGGLE_R_IN + 0.35;
                double bestAcross[4] = {99, 99, 99, 99};
                for (double sd : {-1.0, 1.0}) {
                    double wxp = b.x + c * padX * S - sn * sd * RAM_Y_IN * S;
                    double wyp = b.y + sn * padX * S + c * sd * RAM_Y_IN * S;
                    double fx = wxp / S, fy = (720.0 - wyp) / S;
                    for (int k = 0; k < 4; ++k) {
                        double across = k == 0 ? fx - TOGGLE_INSET_IN : k == 1 ? F - TOGGLE_INSET_IN - fx
                                      : k == 2 ? fy - TOGGLE_INSET_IN : F - TOGGLE_INSET_IN - fy;
                        double along = (k <= 1) ? fy - F / 2 : fx - F / 2;
                        if (std::fabs(along) >= 13.0) continue;
                        bestAcross[k] = std::min(bestAcross[k], across);
                    }
                }
                for (int k = 0; k < 4; ++k) {
                    const double across = bestAcross[k];
                    b.touchTog[k] = across < contact;
                    if (across < contact) {
                        double hx = k == 0 ? -1 : k == 1 ? 1 : 0, hy = k == 2 ? 1 : k == 3 ? -1 : 0;  // toward the wall, sim px
                        double toward = -(c * hx + sn * hy);  // how squarely the back faces that wall
                        if (toward > 0.05) {
                            double back = (contact - across) * S / toward;
                            b.x += c * back; b.y += sn * back;
                            double speedIn = -(b.vL + b.vR) / 2 * toward / S;  // backing speed toward the wall, in/s
                            if (b.ramArmed[k] && speedIn > RAM_MIN_IPS && g.toggle[k] != b.team) {
                                // a robot rolls it the way that brings up its own colour: on a triangular
                                // Toggle each face is next to the other two, so it takes one roll either way
                                g.toggle[k] = b.team;
                                g.togOwner[k] = bi;
                                g.toggleTurn[k] = -1.0;
                                b.ramArmed[k] = false;
                                b.tgHit = true;
                            }
                            if (b.vL + b.vR < 0) slow();
                        }
                    } else if (across > contact + 1.5) {
                        b.ramArmed[k] = true;
                    }
                }
            }
        }
        // Loose pieces: the body shoves them; in the front opening they are free until the claw,
        // whose wheel stacks steer a piece toward the middle and stop it at the grip point
        const double bv = (b.vL + b.vR) / 2, bvx = c * bv, bvy = sn * bv;
        for (int i = 0; i < (int)g.p.size(); ++i) {
            Piece& pc = g.p[i];
            if (!loose(pc) || pc.inCup >= 0 || pc.fz > 0.5) continue;
            double dx = pc.x - b.x, dy = pc.y - b.y;
            if (std::fabs(dx) > 16.0 * S || std::fabs(dy) > 16.0 * S) continue;
            double lx = dx * c + dy * sn, ly = -dx * sn + dy * c;
            double r = floorRadiusIn(pc) * S;
            if (std::fabs(ly) < OPEN_HALF_IN * S && lx > hb - OPEN_DEPTH_IN * S) {
                if (liftIn + 0.35 < 6.5 && lx < grip + 1.6 * S) {
                    double nly = ly * 0.8, nlx = std::max(lx, grip - 1.0 * S);
                    pc.x = b.x + nlx * c - nly * sn;
                    pc.y = b.y + nlx * sn + nly * c;
                    pc.vx = bvx; pc.vy = bvy;
                }
                continue;
            }
            double ox = hb + r - std::fabs(lx), oy = hb + r - std::fabs(ly);
            if (ox <= 0 || oy <= 0) continue;
            double nlx = 0, nly = 0, d;
            if (ox < oy) { nlx = lx >= 0 ? 1.0 : -1.0; d = ox; } else { nly = ly >= 0 ? 1.0 : -1.0; d = oy; }
            double wx = nlx * c - nly * sn, wy = nlx * sn + nly * c;
            pc.x += wx * d; pc.y += wy * d;
            double vn = bvx * wx + bvy * wy, pn = pc.vx * wx + pc.vy * wy;
            if (vn > 0 && pn < vn * 1.1) { pc.vx += wx * (vn * 1.1 - pn); pc.vy += wy * (vn * 1.1 - pn); }
        }
    }
    // Carried pieces ride in the claw at the grip point
    for (const Bot& b : g.bots)
        if (b.on && b.carry >= 0) {
            Piece& pc = g.p[b.carry];
            pc.x = b.x + std::cos(b.h) * BOT_GRIP_IN * S;
            pc.y = b.y + std::sin(b.h) * BOT_GRIP_IN * S;
            if (b.rider >= 0) { g.p[b.rider].x = pc.x; g.p[b.rider].y = pc.y; }
        }
}

// ── CPU driving the player's robot (settings: YOUR ROBOT) ────────────────────
// The computer drives this robot the way a person would: it pushes the controller's sticks and
// buttons, and the robot program (robot/src/main.cpp, driver control) does the rest, with its
// curve, slew, lift speed and claw. It plans like the other robots (botDecide), at its level.
constexpr int PILOT_BTN_L1 = 6, PILOT_BTN_L2 = 7, PILOT_BTN_R1 = 8, PILOT_BTN_R2 = 9;  // PROS controller_digital_e_t
inline void pilotRelease(SimState& s) {
    s.controller.leftY.store(0); s.controller.leftX.store(0);
    s.controller.rightY.store(0); s.controller.rightX.store(0);
    s.controller.buttons.store(0);
}
// Stick value (-1 to 1) that gives this output after the robot code's input curve (DRIVE_CURVE 0.6:
// 0.6 x^3 + 0.4 x), so the CPU gets the power it asks for
inline double pilotUncurve(double y) {
    double a = std::fabs(std::clamp(y, -1.0, 1.0)), lo = 0.0, hi = 1.0;
    for (int i = 0; i < 30; ++i) {
        double m = (lo + hi) / 2;
        if (0.6 * m * m * m + 0.4 * m < a) lo = m; else hi = m;
    }
    return std::copysign(hi, y);
}
inline void pilotUpdate(SimState& s, const std::array<GoalObstacle, 9>& goals, double dt, bool driving) {
    Game& g = game();
    Bot& P = g.pilot;
    if (g.set.pilot <= 0 || g.playerOff) {  // (not while the player's robot sits a tournament match out)
        if (P.on) { P.on = false; pilotRelease(s); }
        return;
    }
    if (!P.on) {
        P = Bot{};
        P.on = true;
        P.team = C_RED;
        P.num = "41658F";
        g.pilotPrevToggle = g.toggle;
    }
    const BotSkill sk = botSkill(g.set.pilot - 1);
    P.focusSet = g.set.focus[3];
    P.scoreYel = g.set.yel[3] != 0;
    { const int nf = botModeNow(g, BOT_N, P); if (nf != P.focus) { P.focus = nf; P.cyc = 0; P.cycT = 0.0; P.task = 0; P.target = -1; } }
    botDropStuck(g, P, BOT_N, dt);
    g_topIps = M_PI * 3.25 * g_wheelRpm / 60.0;  // the player's robot
    const double S = PX_IN, grip = BOT_GRIP_IN * S;
    {
        std::lock_guard<std::mutex> lock(s.pose_mutex);
        P.x = s.pose.x; P.y = s.pose.y; P.h = s.pose.heading;
    }
    P.liftDeg = liftDegFor(g.liftIn);
    // What its claw did since last step
    if (P.carry < 0 && g.held >= 0) {          // took a piece
        P.target = -1; P.task = 0; P.phase = 0;
        P.wait = sk.pause * 0.5;
    } else if (P.carry >= 0 && g.held < 0) {   // let it go
        P.goal = -1;
        if (P.focus == FOCUS_SCORING) { P.wait = 0.0; P.backT = 0.0; }  // the cycle backs out along its own line
        else { P.wait = sk.pause; P.backT = 0.6; P.backTurn = 0.0; }
        ++P.placed;
        if (P.carry >= 0 && P.carry < (int)g.p.size()) g.p[P.carry].placedBy = 3;
        if (P.rider >= 0 && P.rider < (int)g.p.size()) g.p[P.rider].placedBy = 3;
    }
    P.carry = g.held;
    P.rider = g.rider;
    for (int k = 0; k < 4; ++k)
        if (g.toggle[k] != g.pilotPrevToggle[k]) { P.tgHit = true; if (g.toggle[k] == P.team) g.togOwner[k] = 3; }
    g.pilotPrevToggle = g.toggle;
    if (!driving || dt <= 0 || g.menuOpen) { pilotRelease(s); return; }

    BotCmd cmd = botDecide(g, BOT_N, P, sk, goals, dt, -1e6, -1e6, false, true);
    botWatchdog(P, dt, cmd.defending);
    double fwd = cmd.skip ? 0.0 : cmd.fwd, turn = cmd.skip ? 0.0 : cmd.turn;
    // Stuck (pinned while it has a job): back out and leave that piece for a while
    if ((P.stuckT += dt) > 1.5) {
        const bool busy = P.task == 0 ? (P.target >= 0 || P.carry >= 0) : P.phase == 0;
        if (std::hypot(P.x - P.chkX, P.y - P.chkY) < 3.0 * S && P.wait <= 0 && P.backT <= 0 && busy &&
            std::fabs(P.liftTarget - P.liftDeg) < 20.0) {
            P.backT = 0.7;
            P.backTurn = (std::rand() & 1) ? 0.8 : -0.8;
            if (P.target >= 0) { P.skip = P.target; P.skipT = 6.0; P.target = -1; }
        }
        P.stuckT = 0.0; P.chkX = P.x; P.chkY = P.y;
    }
    if (P.skipT > 0 && (P.skipT -= dt) <= 0) P.skip = -1;

    uint32_t btn = 0;
    const double c = std::cos(P.h), sn = std::sin(P.h);
    auto localOf = [&](double px, double py, double& lx, double& ly) {
        lx = (px - P.x) * c + (py - P.y) * sn;
        ly = -(px - P.x) * sn + (py - P.y) * c;
    };
    // Lift: L1 / L2 toward the height its plan wants
    if (P.liftDeg < P.liftTarget - 12.0) btn |= 1u << PILOT_BTN_L1;
    else if (P.liftDeg > P.liftTarget + 12.0 && P.liftDeg > 6.0) btn |= 1u << PILOT_BTN_L2;
    // Claw: run the rollers in near the piece it wants (or at a Loader); out over its goal to let go
    if (P.carry >= 0 && P.goal >= 0) {
        double lx, ly;
        localOf(goals[P.goal].x, goals[P.goal].y, lx, ly);
        if (std::fabs(lx - grip) < 1.2 * S && std::fabs(ly) < 1.4 * S && g.liftIn >= goalTopIn(g, P.goal) - 0.4) {
            btn |= 1u << PILOT_BTN_R2;
            fwd = 0.0;
        }
    } else if (P.carry < 0) {
        bool near = (P.task == 1 && P.phase >= 2) || (P.focus == FOCUS_SCORING && P.cyc == CY_DOCK);
        if (P.task == 0 && P.target >= 0) {
            double lx, ly;
            localOf(g.p[P.target].x, g.p[P.target].y, lx, ly);
            near = std::fabs(lx - grip) < 4.0 * S && std::fabs(ly) < 2.5 * S;
        }
        if (near) btn |= 1u << PILOT_BTN_R1;
    }
    if (P.dropT > 0.0) { btn |= 1u << PILOT_BTN_R2; fwd = 0.0; turn = 0.0; }  // let go of a piece nothing can take
    s.controller.leftY.store((int)std::lround(pilotUncurve(driveNorm(fwd, g_topIps) * sk.stick) * 127.0));
    s.controller.rightX.store((int)std::lround(pilotUncurve(driveNorm(turn, g_topIps) * sk.stick) * 127.0));
    s.controller.leftX.store(0);
    s.controller.rightY.store(0);
    s.controller.buttons.store(btn);
}

// Called every physics step. impactIps: how hard the robot just hit a wall or goal (in/s).
inline GameResult gameUpdate(SimState& s, const RobotConfig& cfg, const std::array<GoalObstacle, 9>& goals,
                             double dt, double impactIps) {
    Game& g = game();
    GameResult res;

    if (!g.inited) {
        g.inited = true;
        {
            std::lock_guard<std::mutex> lock(s.pose_mutex);
            g.sx = s.pose.x;
            g.sy = s.pose.y;
            g.sh = s.pose.heading;
        }
        s.motors[CLAW_PORT].set_position_deg(gripDeg(PIN_W_IN));
        fieldSetup();
        g.prevGapIn = g.gapIn = PIN_W_IN;
    }

    // PLAY on the start screen: a fresh field and a new match in driver control
    if (g.playRequested) {
        g.playRequested = false;
        g.dash = false;
        resetAll(s);
        res.robotMoved = true;
        s.abort_task.store(true);
        s.comp_mode.store(CompMode::OPCONTROL);
    }
    // ── Modes (driver control only, no autonomous): F6 drives from where things are, R resets ──
    CompMode mode = s.comp_mode.load();
    if (g.resetRequested) {
        g.resetRequested = false;
        resetAll(s);
        res.robotMoved = true;
        if (mode != CompMode::DISABLED) {
            s.abort_task.store(true);
            s.comp_mode.store(CompMode::DISABLED);
            mode = CompMode::DISABLED;
        }
    }
    if (g.fastSim && mode == CompMode::DISABLED && !g.matchOver) {  // a simulated match runs until it ends
        s.comp_mode.store(CompMode::OPCONTROL);
        mode = CompMode::OPCONTROL;
    }
    if (mode == CompMode::DISABLED) stopAllMotors(s);  // a disabled robot gets no power
    // Match clock: runs in driver control. With the TIMER showing (SCOREBOARD setting), 0:00 ends the
    // match and disables everything, and F6 after that starts a new 1:45 from where things are. Without
    // it the clock just starts over (with fresh match loads), so the other robots keep their match rhythm.
    if (mode == CompMode::OPCONTROL) {
        if (g.matchOver) { g.matchOver = false; g.matchT = g_matchLen; }
        g.matchT -= dt;
        if (g.skipTo > 0.0) { g.matchT = std::min(g.matchT, g.skipTo); g.skipTo = 0.0; }
        if (g.endRequested) { g.matchT = 0.0; }
        if (g.matchT <= 0.0) {
            if ((g.set.hud & 2) || g.bracketCur >= 0 || g.fastSim || g.endRequested) {
                g.endRequested = false;  // tournament and simulated matches always end at 0:00
                g.matchT = 0.0;
                g.matchOver = true;
                s.abort_task.store(true);
                s.comp_mode.store(CompMode::DISABLED);
                mode = CompMode::DISABLED;
                stopAllMotors(s);
            } else {
                g.matchT = g_matchLen;
                g.loadPins = {{MATCH_LOAD_PINS, MATCH_LOAD_PINS}};
                g.loadCups = {{MATCH_LOAD_CUPS, MATCH_LOAD_CUPS}};
            }
        }
    }
    // After the match, the result shows for a few seconds and then the start screen comes back
    if (g.matchOver && g.bracketCur >= 0) {
        bracketRecord(g);
        if (g.fastSim || g.nextRequested) {  // back to the bracket (straight away when simulated, else when NEXT is pressed)
            g.nextRequested = false;
            g.lastRed = g.red; g.lastBlue = g.blue;
            const bool wasQual = g.bracketCur == QM;
            if (wasQual && g.simLeft > 0) --g.simLeft;
            g.fastSim = false; g.bracketCur = -1; g.playerOff = false;
            g.dash = true; g.dashPage = wasQual ? 6 : 1; g.overT = 0.0;
            if (wasQual && g.simAll && g.autoAll && g.qDone >= qualTotal(g)) { g.autoAll = false; formAlliances(g); g.simAll = true; g.dashPage = 1; bracketStart(g, true); }
            else if (g.simAll && (wasQual ? g.qDone < qualTotal(g) && g.simLeft != 0 : bracketNext(g) >= 0)) { if (wasQual) qualStart(g, true); else bracketStart(g, true); }
            else { g.simAll = false; g.resetRequested = true; }  // the player's robot back on the field
        }
    } else if (g.matchOver && g.fastSim) {  // SIM MATCH finished: its score on the start screen
        g.lastRed = g.red; g.lastBlue = g.blue; g.lastSimulated = true;
        g.fastSim = false; g.playerOff = false;
        g.dash = true; g.dashPage = 0;
        g.resetRequested = true;
    } else if (g.matchOver && !g.dash) {
        if (g.nextRequested) { g.nextRequested = false; g.dash = true; g.lastRed = g.red; g.lastBlue = g.blue; g.lastSimulated = false; }  // waits for NEXT
    } else {
        g.overT = 0.0;
        g.nextRequested = false;
    }
    { setLiftType(g.set.liftBot[3]); static const double RP[4] = {200.0, 360.0, 450.0, 600.0}; g_wheelRpm = RP[std::clamp(g.set.rpmBot[3], 0, 3)]; g_topIps = M_PI * 3.25 * g_wheelRpm / 60.0; }
    // Other robots (settings menu): placed again after a setting change or a reset; they drive only in driver control
    if (g.botsDirty) { g.botsDirty = false; botsSetup(g); }
    botsUpdate(s, cfg, goals, mode == CompMode::OPCONTROL ? dt : 0.0);
    pilotUpdate(s, goals, dt, mode == CompMode::OPCONTROL);

    // ── Match loads: the drive team drops a red/yellow pin or a cup into the red Loader nearest
    // the robot; it falls down the tube onto whatever is already in it ──
    if (g.loadRequest != 0) {
        double rxIn, ryIn;
        {
            std::lock_guard<std::mutex> lock(s.pose_mutex);
            rxIn = s.pose.x / PX_IN; ryIn = (720.0 - s.pose.y) / PX_IN;
        }
        int li = std::hypot(rxIn - LOADERS[0].x, ryIn - LOADERS[0].y) <= std::hypot(rxIn - LOADERS[1].x, ryIn - LOADERS[1].y) ? 0 : 1;
        int& supply = g.loadRequest == 1 ? g.loadPins[0] : g.loadCups[0];  // red's match loads
        if (g.set.infiniteLoads) supply = std::max(supply, 1);
        if (supply > 0 && loaderHasRoom(g, li, g.loadRequest == 1)) {
            --supply;  // pin alone, cup alone, or one of each stacked
            Piece pc{g.loadRequest == 1, LOADERS[li].x * PX_IN, 720.0 - LOADERS[li].y * PX_IN, C_RED, C_YEL};
            // The drive team drops it in any way up: a cup either end first, a pin with either half up
            pc.grayUp = (std::rand() & 1) != 0;
            if (pc.isPin && (std::rand() & 1)) std::swap(pc.top, pc.bot);
            pc.loader = li;
            pc.lz = LOADER_H + 1.0;
            g.loaderStack[li].push_back((int)g.p.size());
            g.p.push_back(pc);
        }
        g.loadRequest = 0;
    }
    // Pieces falling: inside the Loaders down onto the stack, and anything the claw just let go of
    for (int li = 0; li < 4; ++li) {
        auto& ls = g.loaderStack[li];
        for (int k = 0; k < (int)ls.size(); ++k) {
            Piece& pc = g.p[ls[k]];
            double rest = loaderRestIn(g, li, k);
            if (pc.lz > rest || pc.lzv < 0.0) {  // falling, or bouncing back up
                pc.lzv += 386.1 * dt;
                pc.lz -= pc.lzv * dt;
                if (pc.lz <= rest) {
                    pc.lz = rest;
                    pc.lzv = pc.lzv > 12.0 ? -LOADER_BOUNCE_E * pc.lzv : 0.0;  // lands, bounces, settles
                }
            } else {
                pc.lz = rest;
                pc.lzv = 0.0;
            }
        }
    }
    for (Piece& pc : g.p) {
        if (pc.fz <= 0.0 || pc.held) { pc.fz = std::max(0.0, pc.fz); continue; }
        pc.fzv += 386.1 * dt;
        pc.fz -= pc.fzv * dt;
        if (pc.fz <= 0.0) { pc.fz = 0.0; pc.fzv = 0.0; }
    }

    // ── Mechanisms ──
    g.liftIn = dr4bLiftIn(s.motors[LIFT_PORT].get_position_deg());
    g.liftAngle = dr4bAngleRad(s.motors[LIFT_PORT].get_position_deg());
    g.clawDeg = s.motors[CLAW_PORT].get_position_deg();
    g.gapIn = g.held >= 0 ? widthIn(g.p[g.held]) : JAW_OPEN_IN;  // the wheel stacks are a fixed distance apart: they squeeze the held piece

    double rx, ry, rh;
    {
        std::lock_guard<std::mutex> lock(s.pose_mutex);
        rx = s.pose.x; ry = s.pose.y; rh = s.pose.heading;
    }
    // Path trail: the robot centre every 0.1 s (the last minute)
    g.trailT += dt;
    if (g.trailT >= 0.1) {
        g.trailT = 0.0;
        if (g.trail.empty() || std::hypot(g.trail.back().first - rx, g.trail.back().second - ry) > 0.5)
            g.trail.push_back({(float)rx, (float)ry});
        if (g.trail.size() > 600) g.trail.erase(g.trail.begin());
    }
    const double hw = cfg.robot_half_w, hh = cfg.robot_half_h;
    double c = std::cos(rh), sn = std::sin(rh);
    double reach = hw + REACH_IN * PX_IN;
    double cx = rx + c * reach, cy = ry + sn * reach;
    auto toLocal = [&](double x, double y, double& lx, double& ly) {
        double dx = x - rx, dy = y - ry;
        lx = dx * c + dy * sn;
        ly = -dx * sn + dy * c;
    };

    // ── Loader covers. Each Loader's foot opening has a clear cover that stays down, holding the
    // pieces in, and lifts when the robot comes up to it. With the cover up the bottom piece slides
    // out to the opening, where the claw can take it; when the robot leaves the cover drops and the
    // piece slides back in. A piece the robot's body hits as it comes in is shoved out onto the floor
    // instead (the next one then drops to the foot). ──
    for (int li = 0; li < 4; ++li) {
        auto& ls = g.loaderStack[li];
        const double fy = 720.0 - LOADERS[li].y * PX_IN;
        double lx, ly;
        toLocal(loaderOutX(li) * PX_IN, fy, lx, ly);
        const double ddx = std::max(0.0, std::fabs(lx) - hw), ddy = std::max(0.0, std::fabs(ly) - hh);
        const bool near = std::hypot(ddx, ddy) < LOADER_APPROACH_IN * PX_IN || g.botAtLoader[li];
        // The bottom piece only slides out while the robot's front opening is lined up with the foot (or a CPU robot is docked),
        // so a robot coming in at an angle meets a closed cover instead of knocking the piece out onto the floor
        const bool aligned = lx > 0 && std::fabs(ly) < (OPEN_HALF_IN + 1.0) * PX_IN;
        double& gate = g.loaderGate[li];
        gate = near ? std::min(1.0, gate + dt / LOADER_GATE_UP_S) : std::max(0.0, gate - dt / LOADER_GATE_DOWN_S);
        for (int k = 0; k < (int)ls.size(); ++k) {
            Piece& pc = g.p[ls[k]];
            if (k == 1 && pc.isPin != g.p[ls[0]].isPin) { pc.slide = g.p[ls[0]].slide; continue; }  // nested in the bottom one: comes out with it
            if (k > 0 || pc.lz >= 0.3 || pc.lzv < 0.0) { pc.slide = 0.0; continue; }
            pc.slide = (gate >= 0.85 && (aligned || g.botAtLoader[li])) ? std::min(1.0, pc.slide + dt / LOADER_SLIDE_S)
                                    : std::max(0.0, pc.slide - dt / LOADER_SLIDE_S);
        }
        if (ls.empty()) continue;
        Piece& b = g.p[ls.front()];
        if (b.slide < 0.5) continue;
        const double bx = (LOADERS[li].x + (loaderOutX(li) - LOADERS[li].x) * b.slide) * PX_IN;
        double plx, ply;
        toLocal(bx, fy, plx, ply);
        if (std::fabs(ply) < OPEN_HALF_IN * PX_IN && plx > hw - OPEN_DEPTH_IN * PX_IN) continue;  // in the front opening: the claw takes it
        const double pr = floorRadiusIn(b) * PX_IN;
        if (hw + pr - std::fabs(plx) > 0 && hh + pr - std::fabs(ply) > 0) {
            const int idx = ls.front();
            ls.erase(ls.begin());
            Piece& e = g.p[idx];
            e.loader = -1;
            e.slide = 0.0;
            e.lz = 0.0; e.lzv = 0.0;
            e.x = bx; e.y = fy;
        }
    }

    // ── Held piece: follows the claw; must be lifted over a goal's rim to get above it ──
    if (g.held >= 0) {
        int over = -1;
        double overD = 0;
        for (int i = 0; i < 9; ++i) {
            double d = std::hypot(cx - goals[i].x, cy - goals[i].y);
            if (d < goals[i].r) { over = i; overD = d; break; }
        }
        if (over != g.overGoal) {
            g.overGoal = over;
            g.overGoalOk = over >= 0 && g.liftIn >= goalTopIn(g, over) - 0.1;
        }
        if (over >= 0 && !g.overGoalOk) {
            // The piece hits the side of the goal: push the robot back out
            double ux = overD > 1e-6 ? (cx - goals[over].x) / overD : -c;
            double uy = overD > 1e-6 ? (cy - goals[over].y) / overD : -sn;
            double push = goals[over].r - overD + 0.5;
            std::lock_guard<std::mutex> lock(s.pose_mutex);
            s.pose.x += ux * push;
            s.pose.y += uy * push;
            rx = s.pose.x; ry = s.pose.y;
            cx += ux * push; cy += uy * push;
            g.overGoal = -1;
            res.robotMoved = true;
        }
        // A piece grabbed off-centre glides into the grip point (the wheels roll it in) instead of jumping
        const double glide = std::exp(-dt / 0.07);
        g.heldOffX *= glide;
        g.heldOffY *= glide;
        g.p[g.held].x = cx + g.heldOffX;
        g.p[g.held].y = cy + g.heldOffY;
        if (g.rider >= 0) {
            g.p[g.rider].x = cx + g.heldOffX;
            g.p[g.rider].y = cy + g.heldOffY;
        }
        // (The ratchet holds the grip: bumps no longer knock the piece loose.)
        (void)impactIps;
    } else {
        g.overGoal = -1;
    }

    // ── Gripping. Candidates are pieces at the jaws' height between the jaws; a candidate can carry
    // a rider: the piece nested on top of it (a pin standing in a cup, or the top two pieces of a
    // goal stack), so the claw can hold one cup and one pin together.
    //   Closing (R1): a piece is gripped the moment the gap shrinks to its width.
    //   Ratchet latch: with the jaws open and the pawl down, pushing the claw onto a piece until it
    //   sits at the grip point makes the spring fingers click shut on it by themselves.
    struct Cand { int idx, rider, goal, loader; double x, y, bottom; };
    std::vector<Cand> cands;
    for (int i = 0; i < 2; ++i) {  // red alliance goals only: pieces can't be taken off neutral or opposing goals (<S5>)
        auto& st = g.stack[i];
        int n = (int)st.size();
        if (n >= 1) cands.push_back({st[n - 1], -1, i, -1, goals[i].x, goals[i].y, stackBottomIn(i, n - 1)});
        if (n >= 2) cands.push_back({st[n - 2], st[n - 1], i, -1, goals[i].x, goals[i].y, stackBottomIn(i, n - 2)});
    }
    for (int li = 0; li < 4; ++li)
        if (LOADERS[li].c == C_RED && !g.loaderStack[li].empty())  // <SG11c>: only its own alliance's (red) Loaders
            if (g.p[g.loaderStack[li].front()].slide >= 0.95)  // the cover is up and the bottom piece has slid out to the opening
            {
                const auto& ls = g.loaderStack[li];
                const int rider = ls.size() >= 2 && g.p[ls[1]].isPin != g.p[ls[0]].isPin ? ls[1] : -1;  // nested pin and cup
                cands.push_back({ls.front(), rider, -1, li, loaderOutX(li) * PX_IN, 720.0 - LOADERS[li].y * PX_IN, 0.0});
            }
    for (int i = 0; i < (int)g.p.size(); ++i) {
        const Piece& pc = g.p[i];
        if (!loose(pc)) continue;
        int rider = -1;
        if (!pc.isPin && !pc.lying)
            for (int j = 0; j < (int)g.p.size(); ++j)
                if (j != i && g.p[j].isPin && loose(g.p[j]) && !g.p[j].lying &&
                    std::hypot(g.p[j].x - pc.x, g.p[j].y - pc.y) < 0.6 * PX_IN) { rider = j; break; }
        cands.push_back({i, rider, -1, -1, pc.x, pc.y, floorBottomIn(g, i)});
    }

    bool anyInJaws = false;
    if (g.held < 0) {
        const bool rollersIn = g.clawRpm > 20.0;  // wheels pulling in
        const bool latchReady = g.ratchetArmed && g.pawlUp == 0.0;
        int best = -1;
        double bestD = 1e9;
        bool bestLatch = false;
        for (int k = 0; k < (int)cands.size(); ++k) {
            const Cand& cd = cands[k];
            double lx, ly;
            toLocal(cd.x, cd.y, lx, ly);
            // Between the claw's wheel stacks lengthwise (around the grip point, inside the opening)
            bool lengthwise = lx >= hw + (REACH_IN - 1.6) * PX_IN && lx <= hw + (REACH_IN + 2.5) * PX_IN;
            if (lengthwise && std::fabs(ly) < 3.0 * PX_IN) anyInJaws = true;
            if (!lengthwise || std::fabs(g.liftIn - std::max(cd.bottom, LIFT_BASE_IN)) > 1.6) continue;
            double w = widthIn(g.p[cd.idx]);
            bool viaClose = rollersIn && std::fabs(ly) <= (JAW_OPEN_IN / 2) * PX_IN;  // wheels running in take a piece between them
            bool viaLatch = latchReady && g.gapIn > w && std::fabs(ly) < 0.9 * PX_IN &&
                            lx <= hw + (REACH_IN + 0.8) * PX_IN;  // pushed in as far as the grip point
            if (!viaClose && !viaLatch) continue;
            double d = std::hypot(cd.x - cx, cd.y - cy);
            if (d < bestD) { bestD = d; best = k; bestLatch = !viaClose; }
        }
        if (best >= 0) {
            const Cand cd = cands[best];
            if (cd.goal >= 0) {
                auto& st = g.stack[cd.goal];
                st.pop_back();
                if (cd.rider >= 0) st.pop_back();
                g.overGoal = cd.goal;
                g.overGoalOk = true;
            }
            if (cd.loader >= 0) {
                auto& ls = g.loaderStack[cd.loader];
                ls.erase(ls.begin());  // the rest drop down
                if (cd.rider >= 0 && !ls.empty() && ls.front() == cd.rider) ls.erase(ls.begin());  // its nested partner too
            }
            g.held = cd.idx;
            g.rider = cd.rider;
            for (int k : {cd.idx, cd.rider}) {
                if (k < 0) continue;
                g.p[k].held = true;
                g.p[k].goal = -1;
                g.p[k].loader = -1;
                g.p[k].lying = false;  // the claw stands a lying pin (or cup) up, its tip color on top
                g.p[k].tilt = g.p[k].tiltV = 0.0;
                g.p[k].inCup = -1;
                g.p[k].spin = 0.0;
                g.p[k].rollAng = 0.0;
            }
            {   // it keeps where it was; the held-piece update eases it into the grip point
                double offX = cd.x - cx, offY = cd.y - cy, offLen = std::hypot(offX, offY), offMax = 3.0 * PX_IN;
                if (offLen > offMax) { offX *= offMax / offLen; offY *= offMax / offLen; }
                g.heldOffX = offX;
                g.heldOffY = offY;
            }
            if (bestLatch) {
                g.pawlClick = 1.0;  // the ratchet clicks as the piece rolls in between the wheels
            }
        }
    }
    if (g.held < 0 && !anyInJaws && g.pawlUp == 0.0) g.ratchetArmed = true;

    // ── Jaws opening past the piece's width (pawl lifted by R2): it is let go ──
    if (g.held >= 0 && g.pawlUp >= 1.0 && g.clawRpm < -20.0) releaseHeld(g, goals, cx, cy, c, sn);  // pawl lifted and wheels running out: it is let go
    g.prevGapIn = g.gapIn;

    // ── Robot velocity over the field (for pushing pieces) ──
    double robVx = 0.0, robVy = 0.0;
    if (g.prevRValid && dt > 0) {
        robVx = (rx - g.prevRx) / dt;
        robVy = (ry - g.prevRy) / dt;
        if (std::hypot(robVx, robVy) > 150.0 * PX_IN) robVx = robVy = 0.0;  // a reset, not real motion
    }
    g.prevRx = rx; g.prevRy = ry; g.prevRValid = true;

    // ── Pieces as rigid bodies that can tip over (the same model as the robot: a centre of mass
    // over a base edge). A pin stands on its 1.40 in tip and rocks on a 0.70 in radius; a cup stands
    // on its 3.16 in rim (1.58). Both have their centre of mass halfway up (3.25 in). A cup with a pin
    // standing in it is one taller body: centre of mass 5.11 in up (pin 0.20 lb, cup 0.15 lb, estimates).
    const int np = (int)g.p.size();
    for (Piece& pc : g.p)
        if (!loose(pc)) { pc.tilt = pc.tiltV = 0.0; pc.inCup = -1; }
    // A standing pin in a standing cup's top moves, leans and falls with the cup
    std::vector<int> pinIn(np, -1);  // cup -> the pin standing in it
    for (int i = 0; i < np; ++i) {
        Piece& pc = g.p[i];
        pc.inCup = -1;
        if (!pc.isPin || pc.lying || !loose(pc) || pc.fz > 0) continue;
        for (int j = 0; j < np; ++j) {
            const Piece& o = g.p[j];
            if (o.isPin || o.lying || !loose(o) || o.fz > 0 || pinIn[j] >= 0) continue;
            if (std::hypot(o.x - pc.x, o.y - pc.y) < 0.6 * PX_IN) { pc.inCup = j; pinIn[j] = i; break; }
        }
    }
    struct Body { double r, h, k2; };  // base edge radius, COM height, radius of gyration^2 about the COM (in)
    auto bodyOf = [&](int i) -> Body {
        if (g.p[i].isPin) return {PIN_TIP_R_IN, HALF_IN, 3.88};
        if (pinIn[i] >= 0) return {CUP_RIM_R_IN, 5.11, 6.73};
        return {CUP_RIM_R_IN, HALF_IN, 4.50};
    };
    // Every hit a piece takes this step, per unit mass (in/s, sim axes), and the same times the
    // height it hit at. A hit below the centre of mass kicks the base out from under the top; a hit
    // above it pushes the top over. blk: the robot is against it on this side (it can't lean that way).
    std::vector<double> Jx(np, 0.0), Jy(np, 0.0), JZx(np, 0.0), JZy(np, 0.0), blkX(np, 0.0), blkY(np, 0.0);
    auto addHit = [&](int i, double dvx, double dvy, double z) {  // dv in sim px/s
        Jx[i] += dvx / PX_IN; Jy[i] += dvy / PX_IN;
        JZx[i] += dvx / PX_IN * z; JZy[i] += dvy / PX_IN * z;
    };

    // ── The robot body pushes loose pieces along the floor. The body is three rectangles around the
    // front opening (the back plate and the two rails either side of it). A piece overlapping one is
    // moved out along the direction of least overlap by no more than the overlap (at most 2.5 in a
    // step), never snapped to another side, and it takes the speed of the surface that hit it,
    // including the turning of the robot, so it is pushed away smoothly instead of jumping. ──
    {
        double omega = 0.0;  // robot turn rate, rad/s
        if (g.prevRValid && dt > 0) {
            double dh = std::atan2(std::sin(rh - g.prevRh), std::cos(rh - g.prevRh));
            omega = dh / dt;
            if (std::fabs(omega) > 50.0) omega = 0.0;  // a reset, not real motion
        }
        g.prevRh = rh;
        struct Rc { double x0, x1, y0, y1; };
        const double od = OPEN_DEPTH_IN * PX_IN, oh = OPEN_HALF_IN * PX_IN;
        const Rc body[3] = {{-hw, hw - od, -hh, hh}, {hw - od, hw, oh, hh}, {hw - od, hw, -hh, -oh}};
        const double maxCorr = 2.5 * PX_IN;
        for (int i = 0; i < np; ++i) {
            Piece& pc = g.p[i];
            if (!loose(pc) || pc.inCup >= 0) continue;  // a pin in a cup goes with the cup
            const double pr = floorRadiusIn(pc) * PX_IN;
            for (int pass = 0; pass < 2; ++pass) {  // a second pass in case moving out of one rectangle enters another
                double lx, ly;
                toLocal(pc.x, pc.y, lx, ly);
                double bestPen = 0.0, bnx = 0.0, bny = 0.0;
                for (const Rc& r : body) {
                    double qx = std::clamp(lx, r.x0, r.x1), qy = std::clamp(ly, r.y0, r.y1);
                    double dx = lx - qx, dy = ly - qy, d = std::hypot(dx, dy), pen, nx, ny;
                    if (d > 1e-9) {
                        if (d >= pr) continue;
                        pen = pr - d; nx = dx / d; ny = dy / d;
                    } else {  // the centre is inside: leave by the nearest face
                        double l0 = lx - r.x0, l1 = r.x1 - lx, l2 = ly - r.y0, l3 = r.y1 - ly;
                        double m = std::min(std::min(l0, l1), std::min(l2, l3));
                        if (m == l0)      { nx = -1; ny = 0; }
                        else if (m == l1) { nx = 1;  ny = 0; }
                        else if (m == l2) { nx = 0;  ny = -1; }
                        else              { nx = 0;  ny = 1; }
                        pen = m + pr;
                    }
                    if (pen > bestPen) { bestPen = pen; bnx = nx; bny = ny; }
                }
                {   // The goal alignment arms (floor-level plates, 0.1 in thick) funnel a piece toward the middle,
                    // and the claw's two wheel stacks are in the way of anything off-centre while the claw is low.
                    // The middle slot between the wheels stays free so a centred piece rolls in to be gripped.
                    const double pb = (pc.lying ? PIECE_MAX_R_IN : (pc.isPin ? PIN_TIP_R_IN : CUP_RIM_R_IN)) * PX_IN;
                    auto circleHit = [&](double cx0, double cy0, double R, double pe) {
                        double dx = lx - cx0, dy = ly - cy0, d = std::hypot(dx, dy);
                        if (d >= R + pe || d < 1e-9) return;
                        double pen = R + pe - d;
                        if (pen > bestPen) { bestPen = pen; bnx = dx / d; bny = dy / d; }
                    };
                    struct Sg { double ax, ay, bx, by; };
                    const Sg arms[3] = {
                        {hw + ALIGN_BACK_X * PX_IN, -ALIGN_BACK_HALF * PX_IN, hw + ALIGN_MOUTH_X * PX_IN, -ALIGN_MOUTH_HALF * PX_IN},
                        {hw + ALIGN_BACK_X * PX_IN,  ALIGN_BACK_HALF * PX_IN, hw + ALIGN_MOUTH_X * PX_IN,  ALIGN_MOUTH_HALF * PX_IN},
                        {hw + ALIGN_BACK_X * PX_IN, -ALIGN_BACK_HALF * PX_IN, hw + ALIGN_BACK_X * PX_IN,   ALIGN_BACK_HALF * PX_IN}};
                    for (const Sg& sg : arms) {
                        double ex = sg.bx - sg.ax, ey = sg.by - sg.ay;
                        double t = std::clamp(((lx - sg.ax) * ex + (ly - sg.ay) * ey) / (ex * ex + ey * ey), 0.0, 1.0);
                        circleHit(sg.ax + ex * t, sg.ay + ey * t, 0.1 * PX_IN, pb);
                    }
                    if (g.liftIn + 0.35 < PIECE_H_IN)
                        for (double sd : {-1.0, 1.0}) circleHit(hw + REACH_IN * PX_IN, sd * 2.35 * PX_IN, 1.35 * PX_IN, 0.9 * PX_IN);
                }
                if (bestPen <= 0.0) break;
                const double wnx = bnx * c - bny * sn, wny = bnx * sn + bny * c;  // contact normal in field axes
                const double push = std::min(bestPen, maxCorr);
                pc.x += wnx * push;
                pc.y += wny * push;
                if (pass == 0) {
                    ++res.pushed;
                    // Speed of the robot's surface where it touched: its velocity plus its turning
                    const double rxw = pc.x - rx, ryw = pc.y - ry;
                    const double vcx = robVx - omega * ryw, vcy = robVy + omega * rxw;
                    const double vnC = vcx * wnx + vcy * wny, vn = pc.vx * wnx + pc.vy * wny;
                    if (vn < vnC) {  // the surface is moving into it: it is carried along, a little faster than the bumper
                        const double dv = (vnC - vn) + 0.08 * std::max(0.0, vnC);
                        pc.vx += wnx * dv;
                        pc.vy += wny * dv;
                        addHit(i, wnx * dv, wny * dv, ROBOT_HIT_Z_IN);  // the chassis rails hit it low
                    }
                    const double tx = -wny, ty = wnx;  // rubbing drags it a little along the surface
                    const double dvt = (vcx * tx + vcy * ty) - (pc.vx * tx + pc.vy * ty);
                    pc.vx += tx * dvt * 0.25;
                    pc.vy += ty * dvt * 0.25;
                    blkX[i] = wnx; blkY[i] = wny;  // pushed away from the robot
                }
            }
            pc.x = std::clamp(pc.x, pr, cfg.field_w - pr);
            pc.y = std::clamp(pc.y, pr, cfg.field_h - pr);
        }
    }

    // ── Loose pieces slide after a push and stop with friction on the foam; they bump off the
    // walls, the goals and the Loaders instead of passing through ──
    std::vector<double> slideX(np, 0.0), slideY(np, 0.0);  // direction it was sliding (for the friction trip)
    {
        const double decel = FLOOR_MU * G_IN_S2 * PX_IN;  // floor friction, px/s^2 (lying pieces drag more)
        for (int i = 0; i < np; ++i) {
            Piece& pc = g.p[i];
            if (!loose(pc)) { pc.vx = pc.vy = 0.0; continue; }
            if (pc.inCup >= 0) continue;
            double sp = std::hypot(pc.vx, pc.vy);
            if (sp < 1e-3) continue;
            slideX[i] = pc.vx / sp; slideY[i] = pc.vy / sp;
            if (pc.lying) {
                // Lying: across its length it rolls on its collar (pin) or rims (cup) with only rolling
                // resistance; along its length it slides on the foam (harder, it drags on its side)
                const double ux = std::cos(pc.ang), uy = std::sin(pc.ang);
                double va = pc.vx * ux + pc.vy * uy;                      // along the axis: sliding
                double vpx = pc.vx - ux * va, vpy = pc.vy - uy * va;      // across it: rolling
                const double vp = std::hypot(vpx, vpy);
                const double dropA = 1.6 * decel * dt;
                const double dropP = (pc.isPin ? PIN_ROLL_MU : CUP_ROLL_MU) * G_IN_S2 * PX_IN * dt;
                va = std::fabs(va) <= dropA ? 0.0 : va - (va > 0 ? dropA : -dropA);
                const double kp = vp <= dropP ? 0.0 : (vp - dropP) / vp;
                pc.vx = ux * va + vpx * kp;
                pc.vy = uy * va + vpy * kp;
                if (pc.vx == 0.0 && pc.vy == 0.0) continue;
            } else {
                double drop = decel * dt;
                if (drop >= sp) { pc.vx = pc.vy = 0.0; continue; }
                pc.vx *= (sp - drop) / sp;
                pc.vy *= (sp - drop) / sp;
            }
            pc.x += pc.vx * dt;
            pc.y += pc.vy * dt;
            double pr = floorRadiusIn(pc) * PX_IN;
            // Walls: the piece meets the wall at its widest part (collar or rim), at its middle, so
            // a wall stops it without tipping it
            if (pc.x < pr)               { pc.x = pr;               pc.vx = -0.3 * pc.vx; }
            if (pc.x > cfg.field_w - pr) { pc.x = cfg.field_w - pr; pc.vx = -0.3 * pc.vx; }
            if (pc.y < pr)               { pc.y = pr;               pc.vy = -0.3 * pc.vy; }
            if (pc.y > cfg.field_h - pr) { pc.y = cfg.field_h - pr; pc.vy = -0.3 * pc.vy; }
            auto bounceOff = [&](double ox, double oy, double orr, double hitZ) {
                double dx = pc.x - ox, dy = pc.y - oy, d = std::hypot(dx, dy), minD = orr + pr;
                if (d >= minD || d < 1e-6) return;
                double ux = dx / d, uy = dy / d;
                pc.x = ox + ux * minD;
                pc.y = oy + uy * minD;
                double vn = pc.vx * ux + pc.vy * uy;
                if (vn < 0) {
                    pc.vx -= 1.3 * vn * ux; pc.vy -= 1.3 * vn * uy;
                    if (hitZ > 0) addHit(i, -1.3 * vn * ux, -1.3 * vn * uy, hitZ);
                }
            };
            // Goals: a cup's bottom rim hits the goal's base, a pin's collar hits the goal at up to
            // its own middle, so a piece sliding into a goal trips over it
            for (int k = 0; k < 9; ++k)
                bounceOff(goals[k].x, goals[k].y, goals[k].r, pc.lying ? 0.0 : (pc.isPin ? std::min(HALF_IN, GOAL_H_IN[k]) : 0.3));
            for (const LoaderDef& ld : LOADERS) bounceOff(ld.x * PX_IN, 720.0 - ld.y * PX_IN, 2.1 * PX_IN, 0.0);
        }
    }

    // ── Pieces bounce off each other. Shapes from above: a standing piece is a circle at its
    // widest (the 3.16 in collar or rim); a lying pin is three circles along it (thin tips, wide
    // collar in the middle), a lying cup its two rims and the waist between them. Overlaps are
    // pushed apart by mass, and touching pieces moving together get an impulse with restitution
    // PIECE_BOUNCE_E. A hit off a lying piece's middle sets it spinning; a hit low on a standing
    // piece kicks its base out (the tipping below sees it).
    {
        auto massOf = [&](int i) { return g.p[i].isPin ? PIN_MASS_LB : (pinIn[i] >= 0 ? CUP_MASS_LB + PIN_MASS_LB : CUP_MASS_LB); };
        auto k2Of = [&](int i) {  // lying: radius of gyration^2 about the vertical through its middle, px^2
            double r = g.p[i].isPin ? 1.2 : 1.4;
            return (PIECE_H_IN * PIECE_H_IN / 12.0 + r * r / 4.0) * PX_IN * PX_IN;
        };
        struct Circ { double x, y, r; };
        auto shape = [&](int i, Circ* c) -> int {
            const Piece& p = g.p[i];
            if (!p.lying) { c[0] = {p.x, p.y, PIECE_MAX_R_IN * PX_IN}; return 1; }
            double ux = std::cos(p.ang), uy = std::sin(p.ang);
            const double off = (p.isPin ? 2.3 : 2.9) * PX_IN;
            const double rEnd = (p.isPin ? 0.95 : 1.58) * PX_IN, rMid = (p.isPin ? 1.58 : 1.16) * PX_IN;
            c[0] = {p.x - ux * off, p.y - uy * off, rEnd};
            c[1] = {p.x, p.y, rMid};
            c[2] = {p.x + ux * off, p.y + uy * off, rEnd};
            return 3;
        };
        auto inPlay = [&](int i) { const Piece& p = g.p[i]; return loose(p) && p.inCup < 0 && p.fz < 0.5; };
        for (int pass = 0; pass < 2; ++pass)
            for (int i = 0; i < np; ++i) {
                if (!inPlay(i)) continue;
                for (int j = i + 1; j < np; ++j) {
                    if (!inPlay(j)) continue;
                    Piece& A = g.p[i];
                    Piece& B = g.p[j];
                    if (std::fabs(A.x - B.x) > 8.5 * PX_IN || std::fabs(A.y - B.y) > 8.5 * PX_IN) continue;
                    Circ ca[3], cb[3];
                    int na = shape(i, ca), nb = shape(j, cb);
                    for (int a = 0; a < na; ++a)
                        for (int b = 0; b < nb; ++b) {
                            double dx = cb[b].x - ca[a].x, dy = cb[b].y - ca[a].y, d = std::hypot(dx, dy);
                            double minD = ca[a].r + cb[b].r;
                            if (d >= minD || d < 1e-6) continue;
                            const double nx = dx / d, ny = dy / d, over = minD - d;
                            const double mA = massOf(i), mB = massOf(j);
                            // Push apart, the lighter one further
                            A.x -= nx * over * mB / (mA + mB); A.y -= ny * over * mB / (mA + mB);
                            B.x += nx * over * mA / (mA + mB); B.y += ny * over * mA / (mA + mB);
                            for (int k = 0; k < na; ++k) { ca[k].x -= nx * over * mB / (mA + mB); ca[k].y -= ny * over * mB / (mA + mB); }
                            for (int k = 0; k < nb; ++k) { cb[k].x += nx * over * mA / (mA + mB); cb[k].y += ny * over * mA / (mA + mB); }
                            // Contact point and each piece's lever arm to it
                            const double px = ca[a].x + nx * ca[a].r, py = ca[a].y + ny * ca[a].r;
                            const double rAx = px - A.x, rAy = py - A.y, rBx = px - B.x, rBy = py - B.y;
                            const double wA = A.lying ? A.spin : 0.0, wB = B.lying ? B.spin : 0.0;
                            double vAx = A.vx - wA * rAy, vAy = A.vy + wA * rAx;
                            double vBx = B.vx - wB * rBy, vBy = B.vy + wB * rBx;
                            double vrel = (vBx - vAx) * nx + (vBy - vAy) * ny;
                            if (vrel >= 0) continue;  // already separating
                            const double cA = rAx * ny - rAy * nx, cB = rBx * ny - rBy * nx;
                            double denom = 1.0 / mA + 1.0 / mB;
                            if (A.lying) denom += cA * cA / (mA * k2Of(i));
                            if (B.lying) denom += cB * cB / (mB * k2Of(j));
                            const double J = -(1.0 + PIECE_BOUNCE_E) * vrel / denom;
                            A.vx -= nx * J / mA; A.vy -= ny * J / mA;
                            B.vx += nx * J / mB; B.vy += ny * J / mB;
                            if (A.lying) A.spin -= cA * J / (mA * k2Of(i));
                            if (B.lying) B.spin += cB * J / (mB * k2Of(j));
                            // Where it hit, for tipping a standing piece: a lying piece hits low (its
                            // side is 1.58 in up); two standing pieces meet at their widest, halfway up
                            const double z = (A.lying || B.lying) ? PIECE_MAX_R_IN : HALF_IN;
                            if (!A.lying) addHit(i, -nx * J / mA, -ny * J / mA, z);
                            if (!B.lying) addHit(j, nx * J / mB, ny * J / mB, z);
                            if (A.vx != 0 || A.vy != 0) { double sp = std::hypot(A.vx, A.vy); slideX[i] = A.vx / sp; slideY[i] = A.vy / sp; }
                            if (B.vx != 0 || B.vy != 0) { double sp = std::hypot(B.vx, B.vy); slideX[j] = B.vx / sp; slideY[j] = B.vy / sp; }
                        }
                }
            }
        // Lying pieces roll about their own axis as they move across it (no slipping on the collar or
        // rims, 1.58 in radius). A pin coming to rest settles onto one of its eight flat faces.
        for (Piece& p : g.p) {
            if (!loose(p) || !p.lying) continue;
            const double rate = (p.vy * std::cos(p.ang) - p.vx * std::sin(p.ang)) / PX_IN / PIECE_MAX_R_IN;  // rad/s
            p.rollAng += rate * dt;
            if (p.isPin && std::fabs(rate) < 0.6) {
                const double face = 2 * M_PI / PIN_SIDES;
                const double target = std::round(p.rollAng / face) * face;
                p.rollAng += (target - p.rollAng) * std::min(1.0, 8.0 * dt);
            }
        }
        // Lying pieces turn with their spin, which floor friction slows (about 70 rad/s^2)
        for (Piece& p : g.p) {
            if (!loose(p) || !p.lying) { p.spin = 0.0; continue; }
            if (p.spin == 0.0) continue;
            p.ang += p.spin * dt;
            double drop = 70.0 * dt;
            p.spin = std::fabs(p.spin) <= drop ? 0.0 : p.spin - (p.spin > 0 ? drop : -drop);
        }
        for (Piece& p : g.p) {
            if (!loose(p)) continue;
            double pr = floorRadiusIn(p) * PX_IN;
            p.x = std::clamp(p.x, pr, cfg.field_w - pr);
            p.y = std::clamp(p.y, pr, cfg.field_h - pr);
        }
    }

    // ── Lean and fall. Torque per unit mass about the base edge it leans over, at lean a:
    //   hits: (JZ - H J) . d / dt   sliding friction at the floor: mu g H (slide . d)   gravity: -g X
    // with the COM X in from the edge and H above the floor: X = r cos a - h sin a, H = r sin a + h cos a.
    // A pin pushed at the rails' height (above r / mu = 1.56 in) tips over; a lone cup slides upright.
    for (int i = 0; i < np; ++i) {
        Piece& pc = g.p[i];
        if (!loose(pc) || pc.lying || pc.inCup >= 0 || pc.fz > 0) continue;
        const Body bd = bodyOf(i);
        pc.tiltR = bd.r;
        const double Ie = bd.k2 + bd.r * bd.r + bd.h * bd.h;
        const double fr = std::hypot(slideX[i], slideY[i]) > 0.5 ? FLOOR_MU * G_IN_S2 : 0.0;
        const double idt = dt > 0 ? 1.0 / dt : 0.0;
        if (pc.tilt == 0.0 && pc.tiltV == 0.0) {
            // Standing: the push at zero lean, as a vector; it starts to lean only if that beats gravity
            double Fx = (JZx[i] - bd.h * Jx[i]) * idt + fr * bd.h * slideX[i];
            double Fy = (JZy[i] - bd.h * Jy[i]) * idt + fr * bd.h * slideY[i];
            double into = Fx * blkX[i] + Fy * blkY[i];
            if (into < 0) { Fx -= blkX[i] * into; Fy -= blkY[i] * into; }  // can't lean back into the robot
            if (std::hypot(Fx, Fy) <= G_IN_S2 * bd.r) continue;
            pc.tiltDir = std::atan2(Fy, Fx);
            pc.tiltV = 1e-6;
        }
        const double dx = std::cos(pc.tiltDir), dy = std::sin(pc.tiltDir);
        const double jd = Jx[i] * dx + Jy[i] * dy, jzd = JZx[i] * dx + JZy[i] * dy;
        const double sd = slideX[i] * dx + slideY[i] * dy;
        const bool blockedBack = (blkX[i] * dx + blkY[i] * dy) > 0.3;  // leaning back over would go into the robot
        const int n = std::max(1, (int)std::ceil(dt / 0.001));
        const double hs = dt / n;
        bool fell = false;
        for (int k = 0; k < n; ++k) {
            double a = pc.tilt;
            double X = bd.r * std::cos(a) - bd.h * std::sin(a), H = bd.r * std::sin(a) + bd.h * std::cos(a);
            double M = (jzd - H * jd) * idt + fr * H * sd - G_IN_S2 * X;
            pc.tiltV += M / Ie * hs;
            pc.tilt += pc.tiltV * hs;
            if (pc.tilt <= 0.0) {
                pc.tilt = 0.0;
                // Rocks back onto its base; a hard rock carries on over the other edge
                if (pc.tiltV < -1.5 && !blockedBack) { pc.tiltDir += M_PI; pc.tiltV = 0.3 * -pc.tiltV; }
                else pc.tiltV = 0.0;
                break;
            }
            if (pc.tilt >= pieceLandRad(bd.r)) { fell = true; break; }
        }
        if (!fell) continue;
        // Down on its side: it pivoted over its base edge, so its middle lands (edge + half its
        // length) out from where its base was, right where the leaning drawing left it. At the end
        // of a fall the piece is moving down, not sideways, so it keeps only a little speed: a short
        // slide. A pin that rode down in a cup slides on out of the cup's top a bit further.
        const double fallV = pc.tiltV * bd.h * 0.08 * PX_IN;
        const double bx = pc.x, by = pc.y;
        auto lay = [&](Piece& q, double outIn, double vScale) {
            q.lying = true;
            q.ang = pc.tiltDir;
            q.x = bx + dx * outIn * PX_IN;
            q.y = by + dy * outIn * PX_IN;
            q.vx = pc.vx + dx * fallV * vScale;
            q.vy = pc.vy + dy * fallV * vScale;
            q.tilt = q.tiltV = 0.0;
            q.inCup = -1;
            double qr = floorRadiusIn(q) * PX_IN;
            q.x = std::clamp(q.x, qr, cfg.field_w - qr);
            q.y = std::clamp(q.y, qr, cfg.field_h - qr);
        };
        // How far out a point that far up the axis is at the landing lean (base edge pivot)
        const double aL = pc.tilt;
        auto outAt = [&](double along) { return bd.r - bd.r * std::cos(aL) + along * std::sin(aL); };
        const int pin = pinIn[i];
        lay(pc, outAt(HALF_IN), 1.0);
        if (pin >= 0) { lay(g.p[pin], outAt(2 * HALF_IN), 2.0); pinIn[i] = -1; }  // the pin slides out of the top
    }
    // A pin standing in a cup follows it (same place, speed and lean)
    for (int i = 0; i < np; ++i) {
        Piece& pc = g.p[i];
        if (pc.inCup < 0 || pc.lying) continue;
        const Piece& cup = g.p[pc.inCup];
        if (cup.lying) { pc.inCup = -1; continue; }
        pc.x = cup.x; pc.y = cup.y; pc.vx = cup.vx; pc.vy = cup.vy;
        pc.tilt = cup.tilt; pc.tiltV = cup.tiltV; pc.tiltDir = cup.tiltDir; pc.tiltR = cup.tiltR;
    }

    // ── Toggles: rolled one face by backing the passive arms on the DR6B into them (at Toggle height) ──
    double cxs[4] = {-hw, hw, hw, -hw}, cys[4] = {-hh, -hh, hh, hh};
    {
        const double F = FIELD_IN;
        // Both pads in field inches (origin bottom-left): behind the lift's middle tower, RAM_Y_IN to each
        // side. They only reach a Toggle while the lift holds them at Toggle height.
        double padX, padZ;
        ramPadPose(g.liftAngle, hw / PX_IN, padX, padZ);
        const bool padAtToggle = padZ >= RAM_Z_LO && padZ <= RAM_Z_HI;
        double tipF = padX * PX_IN;
        double tips[2][2];
        for (int t = 0; t < 2; ++t) {
            double sy = (t == 0 ? -1.0 : 1.0) * RAM_Y_IN * PX_IN;
            double wx = rx + c * tipF - sn * sy, wy = ry + sn * tipF + c * sy;
            tips[t][0] = wx / PX_IN; tips[t][1] = (720.0 - wy) / PX_IN;
        }
        for (int k = 0; k < 4; ++k) {
            // Closest tip to this Toggle's pivot line, if it is along the Toggle's length
            double best = 99.0;
            for (auto& tp : tips) {
                double across = k == 0 ? tp[0] - TOGGLE_INSET_IN : k == 1 ? F - TOGGLE_INSET_IN - tp[0]
                              : k == 2 ? tp[1] - TOGGLE_INSET_IN : F - TOGGLE_INSET_IN - tp[1];
                double along = (k <= 1) ? tp[1] - F / 2 : tp[0] - F / 2;
                if (padAtToggle && std::fabs(along) < 13.0) best = std::min(best, across);
            }
            const double contact = TOGGLE_R_IN + 0.35;
            bool touch = best < contact;
            double approach = dt > 0 ? (g.ramPrevAcross[k] - best) / dt : 0.0;  // in/s toward the Toggle
            if (touch && g.ramArmed[k] && g.ramPrevAcross[k] < 90 && approach > RAM_MIN_IPS) {
                int f = g.toggle[k] == C_YEL ? 0 : g.toggle[k] == C_RED ? 1 : 2;
                f = (f + 1) % 3;  // yellow -> red -> blue -> yellow
                g.toggle[k] = f == 0 ? C_YEL : f == 1 ? C_RED : C_BLUE;
                g.toggleTurn[k] = -1.0;  // drawn rolling over from the old face
                g.ramArmed[k] = false;
            }
            if (best > contact + 1.5) g.ramArmed[k] = true;  // backed off: the next ram counts
            g.touching[k] = touch;
            g.ramPrevAcross[k] = best;
            // The roll animation eases into place
            double t = g.toggleTurn[k], d = dt / 0.25;
            g.toggleTurn[k] = t > 0 ? std::max(0.0, t - d) : std::min(0.0, t + d);
        }
    }

    // ── Score ──
    {
        // Robots at least partly in the Midfield (<SC6>): each one ending the match there scores 8, and
        // the alliance with more of them owns the Midfield Goal's yellow Pins. The live score shows it
        // as if the match ended now.
        auto inMid = [&](double x, double y, double h, double hw2, double hh2) {
            if (std::fabs(x - 360.0) + std::fabs(y - 360.0) <= MIDFIELD_PX) return true;
            const double cc = std::cos(h), ss = std::sin(h);
            const double fx[4] = {-hw2, hw2, hw2, -hw2}, fy[4] = {-hh2, -hh2, hh2, hh2};
            for (int k = 0; k < 4; ++k)
                if (std::fabs(x + fx[k] * cc - fy[k] * ss - 360.0) + std::fabs(y + fx[k] * ss + fy[k] * cc - 360.0) <= MIDFIELD_PX) return true;
            return false;
        };
        g.midRed = inMid(rx, ry, std::atan2(sn, c), hw, hh) ? 1 : 0;
        g.midBlue = 0;
        for (const Bot& bt : g.bots)
            if (bt.on && inMid(bt.x, bt.y, bt.h, BOT_HALF_IN * PX_IN, BOT_HALF_IN * PX_IN)) (bt.team == C_RED ? g.midRed : g.midBlue)++;
        int r, b;
        scoreField(g, r, b);
        g.red = r + 8 * g.midRed;
        g.blue = b + 8 * g.midBlue;
        // Load Zones (<SG13b>): a robot may only pass through the other alliance's Load Zone. Staying in one
        // for 3 s is a Minor Violation (a warning); a third in a match is a Major Violation: the alliance is
        // Disqualified and loses. (Driver control only; a Disqualified alliance's points still show.)
        if (mode == CompMode::OPCONTROL && dt > 0.0) {
            auto inZone = [&](double x, double y, double h, double hl, double hw2, Col team) {
                // the other alliance's zones: blue's on the right wall for red robots, red's on the left for blue
                const double zx0 = team == C_RED ? (FIELD_IN - LOADZONE_X - 0.7) : 0.0, zx1 = team == C_RED ? FIELD_IN : LOADZONE_X + 0.7;
                const double ext = (std::fabs(std::cos(h)) * hl + std::fabs(std::sin(h)) * hw2) / PX_IN;
                const double eyt = (std::fabs(std::sin(h)) * hl + std::fabs(std::cos(h)) * hw2) / PX_IN;
                const double fx = x / PX_IN, fy = (720.0 - y) / PX_IN;
                const bool xs = fx + ext > zx0 && fx - ext < zx1;
                const bool low = fy - eyt < TILE_IN + 0.7, high = fy + eyt > FIELD_IN - TILE_IN - 0.7;
                return xs && (low || high);
            };
            auto check = [&](double& zt, bool in, Col team, const char* who) {
                if (!in) { zt = 0.0; return; }
                if ((zt += dt) < 3.0) return;
                zt = -3.0;  // the next call needs another 3 s in the zone
                const int a = team == C_RED ? 0 : 1;
                if (++g.minors[a] >= 3) g.dq[a] = true;
                g.violMsg = std::string(team == C_RED ? "RED " : "BLUE ") + who + (g.dq[a] ? ": MAJOR VIOLATION <SG13> - DISQUALIFIED"
                                                                                         : ": MINOR VIOLATION <SG13> LOAD ZONE (WARNING " + std::to_string(g.minors[a]) + ")");
                g.violMsgT = 4.0;
                if (std::getenv("VEXSIM_VIOLATIONS")) if (FILE* vf = std::fopen("vexsim_violations.log", "a")) { std::fprintf(vf, "t %.1f %s %s LOAD ZONE\n", g_matchLen - g.matchT, team == C_RED ? "red" : "blue", who); std::fclose(vf); }
            };
            if (!g.playerOff) {
                check(g.playerZoneT, inZone(rx, ry, std::atan2(sn, c), hw, hh, C_RED), C_RED, "41658F");
                if (g.playerHitCool > 0) g.playerHitCool -= dt;
                bool onBlueGoal = false;  // <SG9>
                for (int gi = 2; gi < 4; ++gi) {
                    double dx = goals[gi].x - rx, dy = goals[gi].y - ry;
                    double lx = dx * c + dy * sn, ly = -dx * sn + dy * c;
                    double ex = lx - std::clamp(lx, -hw, hw), ey = ly - std::clamp(ly, -hh, hh);
                    if (std::hypot(ex, ey) < goals[gi].r + 0.3 * PX_IN) onBlueGoal = true;
                }
                if (onBlueGoal) { if ((g.playerGoalT += dt) > 3.0) { callViolation(g, C_RED, "41658F", "<SG9> OPPOSING GOAL"); g.playerGoalT = -4.0; } }
                else if (g.playerGoalT > 0) g.playerGoalT = 0.0;
                // <SG12>: in the Midfield during the Endgame, stay near starting height
                const bool inMidNow = g.midRed > 0 && inMid(rx, ry, std::atan2(sn, c), hw, hh);
                if (inMidNow && g.matchT <= 10.0 && g.liftIn > LIFT_BASE_IN + 3.0) {
                    if ((g.playerLiftT += dt) > 3.0) { callViolation(g, C_RED, "41658F", "<SG12> LOWER THE LIFT IN THE MIDFIELD"); g.playerLiftT = -5.0; }
                } else if (g.playerLiftT > 0) g.playerLiftT = 0.0;
            }
            for (Bot& bt : g.bots)
                if (bt.on) {
                    const double zBefore = bt.zoneT;
                    check(bt.zoneT, inZone(bt.x, bt.y, bt.h, BOT_HALF_IN * PX_IN, BOT_HALF_IN * PX_IN, bt.team), bt.team, bt.num);
                    if (zBefore >= 0.0 && bt.zoneT < 0.0 && std::getenv("VEXSIM_VIOLATIONS")) if (FILE* vf = std::fopen("vexsim_violations.log", "a")) { std::fprintf(vf, "  at (%.0f,%.0f) h %.0f task %d phase %d target %d goal %d carry %d focus %d cyc %d wait %.1f backT %.1f\n", bt.x / PX_IN, (720.0 - bt.y) / PX_IN, bt.h * 57.3, bt.task, bt.phase, bt.target, bt.goal, bt.carry, bt.focus, bt.cyc, bt.wait, bt.backT); std::fclose(vf); }
                }
        }
        if (g.violMsgT > 0.0) g.violMsgT -= dt;
    }
    return res;
}

} // namespace sim
