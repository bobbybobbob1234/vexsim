#define _USE_MATH_DEFINES
#include "pros/imu.h"
#include "pros/imu.hpp"
#include "sim/SimState.hpp"
#include <cmath>

static constexpr double PI = 3.14159265358979323846;

// The sim's pose.heading is in screen terms: radians, 0 = right (+x), and because screen y
// points down, increasing heading turns the robot clockwise on screen. That matches the VEX
// inertial sensor (clockwise positive), so compass heading = heading in degrees + 90
// (0 = up the field). Taring or setting the sensor only shifts what it reports; it never
// moves the simulated robot.
static double headingOffsetDeg  = 0.0;
static double rotationOffsetDeg = 0.0;

static double wrap360(double d) {
    d = std::fmod(d, 360.0);
    return d < 0 ? d + 360.0 : d;
}

static double rawCompassDeg() {
    auto& state = sim::SimState::get();
    std::lock_guard<std::mutex> lock(state.pose_mutex);
    return state.pose.heading * (180.0 / PI) + 90.0;
}

static double rawRotationDeg() {
    auto& state = sim::SimState::get();
    std::lock_guard<std::mutex> lock(state.pose_mutex);
    return state.pose.heading_accumulated * (180.0 / PI);
}

namespace pros { namespace c {

int32_t imu_reset(uint8_t port) {
    // No-op: calibration is instant in simulation
    return 1;
}

int32_t imu_set_data_rate(uint8_t port, uint32_t rate) { return 1; }

double imu_get_rotation(uint8_t port) { return rawRotationDeg() + rotationOffsetDeg; }

double imu_get_heading(uint8_t port) { return wrap360(rawCompassDeg() + headingOffsetDeg); }

double imu_get_roll(uint8_t port)  { return 0.0; }
double imu_get_pitch(uint8_t port) { return 0.0; }
double imu_get_yaw(uint8_t port)   { return imu_get_heading(port); }

imu_gyro_s_t imu_get_gyro_rate(uint8_t port) {
    return {0.0, 0.0, 0.0};
}

imu_accel_s_t imu_get_accel(uint8_t port) {
    return {0.0, 0.0, 1.0}; // 1g down
}

imu_status_e_t imu_get_status(uint8_t port) {
    return static_cast<imu_status_e_t>(0); // ready
}

int32_t imu_set_heading(uint8_t port, double target) {
    headingOffsetDeg = target - rawCompassDeg();
    return 1;
}

int32_t imu_set_rotation(uint8_t port, double target) {
    rotationOffsetDeg = target - rawRotationDeg();
    return 1;
}

int32_t imu_tare_heading(uint8_t port)  { return imu_set_heading(port, 0.0); }
int32_t imu_tare_rotation(uint8_t port) { return imu_set_rotation(port, 0.0); }

int32_t imu_tare_pitch(uint8_t port) { return 1; }
int32_t imu_tare_roll(uint8_t port)  { return 1; }
int32_t imu_tare_yaw(uint8_t port)   { return imu_tare_heading(port); }

int32_t imu_tare(uint8_t port) {
    imu_tare_heading(port);
    imu_tare_rotation(port);
    return 1;
}

}} // namespace pros::c

namespace pros { inline namespace v5 {

Imu::Imu(uint8_t port) : Device(port) {}

int32_t Imu::reset() const             { return pros::c::imu_reset(_port); }
int32_t Imu::set_data_rate(uint32_t r) const { return pros::c::imu_set_data_rate(_port, r); }
double  Imu::get_rotation() const      { return pros::c::imu_get_rotation(_port) - rotation_offset_; }
double  Imu::get_heading() const       { return pros::c::imu_get_heading(_port); }
double  Imu::get_roll() const          { return 0.0; }
double  Imu::get_pitch() const         { return 0.0; }
double  Imu::get_yaw() const           { return pros::c::imu_get_yaw(_port); }
imu_gyro_s_t  Imu::get_gyro_rate() const { return pros::c::imu_get_gyro_rate(_port); }
imu_accel_s_t Imu::get_accel() const  { return pros::c::imu_get_accel(_port); }
ImuStatus Imu::get_status() const { return ImuStatus::ready; }
bool    Imu::is_calibrating() const    { return false; }
int32_t Imu::tare_heading() const      { return pros::c::imu_tare_heading(_port); }
int32_t Imu::tare_rotation() const     { return pros::c::imu_tare_rotation(_port); }
int32_t Imu::tare_pitch() const        { return 1; }
int32_t Imu::tare_roll() const         { return 1; }
int32_t Imu::tare_yaw() const          { return pros::c::imu_tare_yaw(_port); }
int32_t Imu::tare() const              { return pros::c::imu_tare(_port); }
int32_t Imu::set_heading(double t) const   { return pros::c::imu_set_heading(_port, t); }
int32_t Imu::set_rotation(double t) const  { return pros::c::imu_set_rotation(_port, t); }

}} // namespace pros::v5
