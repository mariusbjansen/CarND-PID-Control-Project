#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Saturation of control
  */
  double Cmin;
  double Cmax;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_in, double Ki_in, double Kd_in, double Cmin_in,
            double Cmax_in);

  /*
  * Control the output variable of the PID controller.
  */
  double Control(double error);
};

#endif /* PID_H */
