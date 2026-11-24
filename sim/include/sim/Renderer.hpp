#pragma once
#include <SDL2/SDL.h>
#include "SimState.hpp"
#include "Physics.hpp"

namespace sim {

constexpr int SIDEBAR_W = 300;  // controls sidebar on the right edge of the window, pixels

class Renderer {
public:
    Renderer(SDL_Renderer* ren, int field_w, int field_h, const RobotConfig& cfg);
    void draw(const SimState& state);

private:
    SDL_Renderer* ren_;
    int fw_, fh_;
    RobotConfig cfg_;

    void drawBackground();
    void drawField();
    void drawRobot(double x, double y, double heading);
    void drawRobotTop(double x, double y, double heading, double liftAngle, double wheelRotL, double wheelRotR,
                      double clawDeg, int team);
    void drawHUD(const SimState& state, double x, double y, double heading);
    void drawCursorOverlay(const SimState& state);

    static void setColor(SDL_Renderer* r, uint8_t red, uint8_t g, uint8_t b, uint8_t a = 255);
};

} // namespace sim
