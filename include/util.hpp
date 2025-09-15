#pragma once
#include "vex.h"

struct PID {
  double kP{0}, kI{0}, kD{0};
  double i{0}, last{0}, limitI{0};
  double step(double err, double dt);
  void reset();
};

double clamp(double x, double lo, double hi);
double sgn(double x);
