# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

### Task
In this project the goal was to implement a PID controller, run it in the Udacity simulator and tune the parameters.

### Effects of P, I and D

<img src="PID_en.svg.png" width="600">
Source: https://en.wikipedia.org/wiki/PID_controller

The PID controller consists of an P- a D- and an I- part.

* Term P is proportional to the current value of the error (in our case the cross track error *cte*). It is basically a factor. If the error is high you multiply the error with the tuned factor for P gain (Kp) and that's it. If the error is low or 0 there is little or nothing to do for the P part
* Term D  is proportional to the derivative (rate of change) of the system. When the error is still high but the system already changes in the direction of minimizing the error, the D part prevents the system from overshooting and therefore oscillating to much.
* Term I accounts for the integrated error and helps delivering stationary accuracy. Example: When the error is once 0 the P part is 0. If the system requires some constant value for the control variable the I part will take this into account because it is integrating over the error.

Comments

* It is possible to just pass with a P controller and a low velocity (throttle) value.
* The D part helps preventing oscillating to much and the velocity can be increased.
* The I part in this specific project did not make to much sense because P and D had already enough to do. You can imagine driving on a straight road with constant bending, in this scenario the I part would be perfect to reach stationary accuracy.

#### Tuning
I just tuned P with step size 0.1 and then D with also step size 0.1. And then went to 0.05 steps. Eventually I set some value for I but it's not really necessary. There are many ways like twiddle which was introduced by Sebastian but also some old fashioned methods from the 1940s like Ziegler–Nichols method: Set Ki and Kd to zero. And then increase P until it reaches the ultimate gain and oscillations start. Then you have a lookup table for oscillation period T_oscillation to set I and/or D. I would say I did it the Ziegler-Nichols way even if I did not have a specific value for T_oscillation and it was more a trial and error.

#### Throttle control
I decided to say basically I want to have throttle of 0.2 by default. Then there are two cases
* If the sum of the last four and current steering angle is low, I want to apply linearly extra throttle till maximum throttle
* If there is a critical situation (judged by cte > 0.9 and speed > 18), I want maximum brake.

```c
// introduce moving sum of steering values
static double s[5] = {0};
s[4] = s[3];
s[3] = s[2];
s[2] = s[1];
s[0] = fabs(steer_value);
double sum = s[0] + s[1] + s[2] + s[3] + s[4];
double throttle = 0.2;
// sum steering values below threshold -> accelerate (lin. function)
if (fabs(sum) < 0.1) {
  throttle += -8.0 * fabs(sum) + 0.8;
}
// High cross track error with exceeding speed threshold -> full brake
if ((fabs(cte) > 0.9) && (speed > 18.0)) {
  throttle = -1.0;
}
```

I decdided actively against introducing a new PID controller for throttle because I don't know the target value for speed. That's different than knowing the target value for cte (which is 0) and here it makes perfectly sense to introduce a PID controller for steering.

### Code
I refactored methods *UpdateError* and *TotalError* to just *Control* and incorporated saturation of the control variable in the controller class iteself.