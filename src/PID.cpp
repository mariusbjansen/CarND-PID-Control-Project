#include "PID.h"
#include <algorithm>

using namespace std;

PID::PID() {}
PID::~PID() {}

void PID::Init(double Kp_in, double Ki_in, double Kd_in, double Cmin_in,
               double Cmax_in) {
  // controller parameters
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;
  // saturation of control variable
  Cmin = Cmin_in;
  Cmax = Cmax_in;
}

double PID::Control(double error) {
  // individual error contributions
  d_error = error - p_error;
  i_error += error;
  p_error = error;
  // PID control formula
  double control_output = -1.0 * (Kp * p_error + Kd * d_error + Ki * i_error);
  // saturation
  control_output = max(Cmin, min(control_output, Cmax));

  return control_output;
}
