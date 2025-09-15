#include "robot-config.h"

using namespace vex;

brain       Brain;
controller  Master{ controllerType::primary };

// Gear cartridges and reversals set for 6-motor tank on 3-wire splits
// TODO Change ports, reversals, and gearsets!!

motor  L1{ PORT1,  gearSetting::ratio6_1,  false };
motor  L2{ PORT2,  gearSetting::ratio6_1,  true  };
motor  L3{ PORT3,  gearSetting::ratio6_1,  false };
motor  R1{ PORT8,  gearSetting::ratio6_1,  true  };
motor  R2{ PORT9,  gearSetting::ratio6_1,  false };
motor  R3{ PORT10, gearSetting::ratio6_1,  true  };

inertial IMU{ PORT11 };

static void setBrake(brakeType b) {
  L1.setBrake(b); L2.setBrake(b); L3.setBrake(b);
  R1.setBrake(b); R2.setBrake(b); R3.setBrake(b);
}

void configure() {
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
  setBrake(brakeType::coast);

  L1.setVelocity(0, percent); L2.setVelocity(0, percent); L3.setVelocity(0, percent);
  R1.setVelocity(0, percent); R2.setVelocity(0, percent); R3.setVelocity(0, percent);

  IMU.calibrate();
  while (IMU.isCalibrating()) wait(10, msec);
}
