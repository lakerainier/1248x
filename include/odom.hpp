#pragma once
#include "vex.h"

// differential odometry w/ integrated motor encoders and IMU heading
// TODO match track/wheel measurements

struct OdomConfig {
  double wheelDiam;     // in
  double gearRatio;     // motor revs / wheel rev
  double trackWidth;    // in - distance between left and right wheel midline
};

struct Pose {
  double x{0};
  double y{0};
  double th{0};         // rad
};

class Odom {
public:
  Odom(vex::motor_group& left, vex::motor_group& right, vex::inertial& imu, const OdomConfig& cfg);
  void reset(const Pose& p);
  void task();
  Pose pose() const;
  void setHeadingHold(bool on);
private:
  vex::motor_group& L;
  vex::motor_group& R;
  vex::inertial&    IMU;
  OdomConfig        cfg;
  double lastL{0}, lastR{0};
  Pose   p{};
  bool   headingHold{false};
};
