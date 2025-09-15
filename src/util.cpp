#include "util.hpp"

double PID::step(double err, double dt) {
  if (limitI > 0) {
    double newI = i + err * dt;
    if (newI >  limitI) newI =  limitI;
    if (newI < -limitI) newI = -limitI;
    i = newI;
  } else {
    i += err * dt;
  }
  double d = (err - last) / dt;
  last = err;
  return kP * err + kI * i + kD * d;
}

void PID::reset() { i = 0; last = 0; }

double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

double sgn(double x) { return (x > 0) - (x < 0); }
