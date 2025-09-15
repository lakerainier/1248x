#include "drivetrain.hpp"
#include <cmath>

using namespace vex;

static double rad2deg(double r){ return r * 180.0 / M_PI; }
static double degNorm(double d) {
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

Drivetrain::Drivetrain(motor_group& left, motor_group& right, inertial& imu, Odom& od)
: L(left), R(right), IMU(imu), odom(od) {}

void Drivetrain::setLR(double lpct, double rpct) {
  lpct = clamp(lpct, -maxPct, maxPct);
  rpct = clamp(rpct, -maxPct, maxPct);
  L.spin(directionType::fwd, lpct, percent);
  R.spin(directionType::fwd, rpct, percent);
}

void Drivetrain::arcade(double fwd, double yaw) {
  setLR(fwd + yaw, fwd - yaw);
}

void Drivetrain::tank(double l, double r) {
  setLR(l, r);
}

void Drivetrain::setBrake(brakeType b) {
  L.setBrake(b);
  R.setBrake(b);
}

void Drivetrain::setMaxPct(double p) { maxPct = clamp(p, 0, 100); }

void Drivetrain::stop() { L.stop(brakeType::hold); R.stop(brakeType::hold); }

void Drivetrain::setGains(const DriveGains& g) { gains = g; }

void Drivetrain::update(double dt) {
  // TODO: feedforward??
  (void)dt;
}

void Drivetrain::driverArcadeWithSlew(double axisF, double axisR, const Slew& s, double dt) {
  double targetL = clamp(axisF + axisR, -100, 100);
  double targetR = clamp(axisF - axisR, -100, 100);

  auto step = [&](double cmd, double last) {
    double delta = cmd - last;
    double lim = (std::abs(cmd) < std::abs(last) ? s.decel : s.accel) * dt;
    delta = clamp(delta, -lim, lim);
    return last + delta;
  };

  lastCmdL = step(targetL, lastCmdL);
  lastCmdR = step(targetR, lastCmdR);
  setLR(lastCmdL, lastCmdR);
}

bool Drivetrain::driveInches(double inches, double maxP, double tol, double timeout) {
  gains.dist.reset();
  gains.turn.reset();

  // reference is current pose forward direction
  const double start = (L.position(rev) + R.position(rev)) * 0.5;
  const double circ  = M_PI * 3.25;     // default wheel
  const double ratio = 1.0;             // TODO: change if external gearing

  int t = 0;
  while (t < timeout) {
    double revNow = (L.position(rev) + R.position(rev)) * 0.5;
    double traveled = (revNow - start) * circ / ratio;
    double err = inches - traveled;

    double headingErr = IMU.rotation(); // minimize drift
    double uD = gains.dist.step(err, 0.02);
    double uT = gains.turn.step(-headingErr, 0.02);

    double l = clamp(uD + uT, -maxP, maxP);
    double r = clamp(uD - uT, -maxP, maxP);
    setLR(l, r);

    if (std::fabs(err) < tol && std::fabs(headingErr) < 1.5) break;

    wait(20, msec);
    t += 20;
  }
  stop();
  return t < timeout;
}

// if ur reading this ur skibidi, anyways hi

bool Drivetrain::turnDegrees(double deg, double maxP, double tol, double timeout) {
  gains.turn.reset();
  const double start = IMU.rotation();
  int t = 0;
  while (t < timeout) {
    double cur = IMU.rotation();
    double err = degNorm((start + deg) - cur);
    double u = gains.turn.step(err, 0.02);
    double v = clamp(u, -maxP, maxP);
    setLR(v, -v);
    if (std::fabs(err) < tol) break;
    wait(20, msec);
    t += 20;
  }
  stop();
  return t < timeout;
}

bool Drivetrain::driveToXY(double x, double y, double thDeg, double maxP, double timeout) {
  gains.dist.reset();
  gains.turn.reset();

  int t = 0;
  while (t < timeout) {
    Pose q = odom.pose();
    double dx = x - q.x;
    double dy = y - q.y;

    double targetTh = rad2deg(std::atan2(dy, dx));
    double headingErr = degNorm(targetTh - IMU.rotation());

    double distErr = std::sqrt(dx*dx + dy*dy);

    double uD = gains.dist.step(distErr, 0.02);
    double uT = gains.turn.step(headingErr, 0.02);

    double l = clamp(uD + uT, -maxP, maxP);
    double r = clamp(uD - uT, -maxP, maxP);
    setLR(l, r);

    if (distErr < 1.0) break;

    wait(20, msec);
    t += 20;
  }
  stop();
  if (t >= timeout) return false;

  // heading trim
  return turnDegrees(thDeg - IMU.rotation(), 60, 1.0, 1500);
}
