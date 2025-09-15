#pragma once
#include "vex.h"
#include "util.hpp"
#include "odom.hpp"

struct DriveGains {
  PID dist{0.9, 0.0, 4.0, 0.2};     // TODO: tune on robot
  PID turn{2.0, 0.0, 10.0, 0.2};    // TODO: tune on robot
};

struct Slew {
  double accel{200}; // pct / second
  double decel{400}; // pct / second
};

class Drivetrain {
public:
  Drivetrain(vex::motor_group& left, vex::motor_group& right, vex::inertial& imu, Odom& odom);

  void arcade(double fwd, double yaw);
  void tank(double l, double r);

  void setBrake(vex::brakeType b);
  void setMaxPct(double p);
  void stop();
  bool driveInches(double inches, double maxPct = 80, double tol = 0.5, double timeout = 2000);
  bool turnDegrees(double deg, double maxPct = 70, double tol = 1.0, double timeout = 2000);
  bool driveToXY(double x, double y, double thDeg, double maxPct = 80, double timeout = 4000);

  // Teleop
  void driverArcadeWithSlew(double axisF, double axisR, const Slew& s, double dt);

  void update(double dt);

  void setGains(const DriveGains& g);

private:
  vex::motor_group& L;
  vex::motor_group& R;
  vex::inertial&    IMU;
  Odom&             odom;

  DriveGains gains{};
  double maxPct{100.0};
  double lastCmdL{0}, lastCmdR{0};

  void setLR(double lpct, double rpct);
};
