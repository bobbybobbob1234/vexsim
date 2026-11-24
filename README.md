# VexSim

A keyboard simulator for a VEX V5 robot running the **V5RC Override** (2026-27) game. It compiles your PROS robot program (`src/` and `include/` of a PROS project) against stub PROS and LemLib libraries and runs it on a simulated field with physics, the game rules and CPU opponents. The 3D view, tournament mode, stats and the standalone-app packaging are additions on top of the original simulator.

This is a modified copy of the **VEX Sim** VS Code extension by avinashselvakumar (`avinashselvakumar.vex-sim`, GPL-3.0-only). It keeps that license: see `LICENSE`. It is a fan project and is not affiliated with or endorsed by VEX Robotics. The VEX logo is not included.

## What it does

- Runs your `initialize()` / `opcontrol()` code with simulated motors, an inertial sensor and a controller (the stubs are in `stubs/`; LemLib is linked from a LemLib source folder).
- The Override field and rules: goals, pins and cups, Toggles, Loaders and match loads, Load Zones, the Midfield bonus, a 1:45 match clock, and referee calls (Load Zone, goal contact, pinning, lift in the Midfield, protected robots).
- A drivetrain with traction limits, drift and tipping, a DR6B / DR4B / cascade lift, a ratcheting roller claw and passive toggle arms.
- CPU robots that score, load, flip Toggles and defend, and adapt to the score; per-robot rpm, lift and skill settings.
- A start screen, a 40-team qualification and elimination tournament drawn from a pool of 60 teams, a bracket, per-team stats, and tournament and all-time leaderboards (`vexsim_stats.jsonl`).
- 2D map, several 3D cameras, a settings menu (Tab), and a controls sidebar.

## Build (Windows)

You need MSYS2 with the UCRT64 toolchain (`gcc`, `make`, `lld`, SDL2) and CMake.

```
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ^
  -DROBOT_SRC_DIR=<your PROS project>/src ^
  -DROBOT_INCLUDE_DIR=<your PROS project>/include ^
  -DLEMLIB_DIR=<LemLib template folder>
cmake --build build --target vex_sim
```

`LEMLIB_DIR` is a LemLib PROS template (https://github.com/LemLib/LemLib, MIT), the folder that contains its `include/` and `src/`. Without `ROBOT_SRC_DIR` the small sample program in `robot/` is used. `CMakePresets.json` has a ready `msys2-ucrt64` preset. If `sim/vex.ico` exists it is embedded as the app icon (the repository does not ship one).

## Run

```
build/sim/vex_sim.exe --wheel-radius 1.625 --track-width 13 --max-rpm 600 --gear-ratio 0.75 ^
  --left-ports 11,-2,-1 --right-ports -20,9,10 --robot-width 18 --robot-height 18 ^
  --start-x -61.2 --start-y 37.55 --start-heading 90
```

The ports are your drive motor ports (negative = reversed). The window opens on the start screen: choose the robots and settings with the mouse, then PLAY (or Enter).

| Key | What it does |
| --- | --- |
| W / S | drive forward / back (left stick) |
| A / D | turn (right stick) |
| Up / Down | lift up / down (L1 / L2); E and Q also work |
| Right / Left | claw close / open (R1 / R2); Space and F also work |
| Z / X | the drive team drops a pin / a cup into the nearest red Loader |
| 1 to 9, 0, C / V | pick a 3D camera / step through them |
| Tab | settings menu (arrow keys and Enter) |
| F6 / F7 | driver control / disable |
| F8 / F9 | end the match / jump to 15 s left |
| R | reset the field |
| F11, Esc | full screen, quit |

## Test switches

Environment variables for headless runs and screenshots: `VEXSIM_TRACE`, `VEXSIM_FIXED_DT=0.02`, `VEXSIM_AUTOPLAY`, `VEXSIM_PILOT` (the CPU drives your robot), `VEXSIM_START=<seconds>`, `VEXSIM_LIFT` (0 cascade, 1 DR4B, 2 DR6B, one digit or one per robot), `VEXSIM_RPM`, `VEXSIM_FOCUS`, `VEXSIM_CAM`, `VEXSIM_HUD`, `VEXSIM_INFINITE`, `VEXSIM_VIOLATIONS` (logs referee calls to `vexsim_violations.log`), `VEXSIM_PAGE=<n>` (open a start-screen page), `VEXSIM_BRACKET` (+ `VEXSIM_QMATCHES`, `VEXSIM_MIX`, `VEXSIM_TEAMLIFT`) to simulate a whole event, `VEXSIM_QSIMN=<n>`, and `VEXSIM_SHOT=<file.bmp>` (+ `VEXSIM_SHOT_T`, `VEXSIM_SHOT_DASH`) to save the window.

## Layout

- `sim/`: the simulator (`Game.hpp` holds the game, CPU and tournament logic; `Physics`, `Renderer`, `View3D.hpp`, `InputHandler`, `UiText`).
- `stubs/`: stub PROS API (motors, IMU, controller, tasks, LCD).
- `lemlib/`: the shim that builds LemLib against the stubs.
- `robot/`: a sample robot program, used when no PROS project is given.

## Known limits

There is no autonomous mode in this copy (the game starts in driver control). Physics numbers, motor torque and the CPU's strategy are approximations, and the rules follow Override game manual v1.1 (check the current manual). The simulator is Windows-oriented (it uses GDI for text and Windows APIs for the console window).
