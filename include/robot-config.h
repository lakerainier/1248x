#pragma once
#include "vex.h"

extern vex::brain       Brain;
extern vex::controller  Master;

extern vex::motor  L1;
extern vex::motor  L2;
extern vex::motor  L3;
extern vex::motor  R1;
extern vex::motor  R2;
extern vex::motor  R3;

extern vex::inertial IMU;

void configure();
