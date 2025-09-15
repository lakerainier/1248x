#include "vex.h"
#include "robot-config.h"
#include "drivetrain.hpp"
#include "odom.hpp"

using namespace vex;

competition Competition;

static motor_group Left{ L1, L2, L3 };
static motor_group Right{ R1, R2, R3 };

static OdomConfig odCfg{ 3.25, 1.0, 12.0 }; // TODO: actually measure...
static Odom*       gOdom   = nullptr;
static Drivetrain* gDrive  = nullptr;

static bool skills = false;

static int odomTask() {
  while (true) {
    gOdom->task();
    wait(10, msec);
  }
  return 0;
}

void pre_auton() {
  configure();
  gOdom  = new Odom(Left, Right, IMU, odCfg);
  gOdom->reset({0,0,0});
  gDrive = new Drivetrain(Left, Right, IMU, *gOdom);
  task t1(odomTask);
}

// auton with random values
void autonomous() {
  // preload clear
  gDrive->driveInches(18, 70);
  gDrive->turnDegrees(45, 60);
  gDrive->driveInches(20, 70);

  // go to point then face random goal
  gDrive->driveToXY(24, 12, 90, 70);
}

void usercontrol() {
  Slew slew{ 250, 500 };
  while (true) {
    double f = Master.Axis3.position(percent);
    double r = Master.Axis1.position(percent);

    gDrive->driverArcadeWithSlew(f, r, slew, 0.02);
    gDrive->update(0.02);

    if (Master.ButtonX.pressing()) gDrive->setBrake(brakeType::hold);
    if (Master.ButtonB.pressing()) gDrive->setBrake(brakeType::coast);
    if (Master.ButtonY.pressing()) IMU.calibrate();

    wait(20, msec);
  }
}

int main() {
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(autonomous);
  pre_auton();
  while (true) wait(100, msec);
}
