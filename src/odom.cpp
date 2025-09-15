#include "odom.hpp"
#include <cmath>

using namespace vex;

static double deg2rad(double d) { return d * M_PI / 180.0; }

Odom::Odom(motor_group& left, motor_group& right, inertial& imu, const OdomConfig& c)
: L(left), R(right), IMU(imu), cfg(c) {}

void Odom::reset(const Pose& start) {
  p = start;
  L.resetPosition();
  R.resetPosition();
  lastL = 0;
  lastR = 0;
}

Pose Odom::pose() const { return p; }

void Odom::setHeadingHold(bool on) { headingHold = on; }

void Odom::task() {
  const double ticksL = L.position(rev);
  const double ticksR = R.position(rev);

  const double wheelCirc = M_PI * cfg.wheelDiam;
  const double dL = (ticksL - lastL) * wheelCirc / cfg.gearRatio;
  const double dR = (ticksR - lastR) * wheelCirc / cfg.gearRatio;

  lastL = ticksL;
  lastR = ticksR;

  const double dS = 0.5 * (dL + dR);
  double th = deg2rad(IMU.rotation());

  // integration!! (around heading)
  p.x += dS * std::cos(th);
  p.y += dS * std::sin(th);
  p.th = th;
}
