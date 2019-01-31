#include "PID.h"
#include <stdio.h>
#include <float.h>
#include <math.h>

PID::PID()
{
  p_error_ = 0;
  i_error_ = 0;
  d_error_ = 0;

  prev_cte_ = 0;
  prev_tp = chrono::system_clock::now();
}

PID::~PID()
{
}

void PID::Init(double Kp, double Ki, double Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_[0] = Kp;
  p_[1] = Ki;
  p_[2] = Kd;
  d_[0] = 0.05;
  d_[1] = 0.00005;
  d_[2] = 0.005;
  best_error_ = FLT_MAX;
  total_error_ = 0;
}

double PID::UpdateError(double cte)
{
  static int steps = 0;

  auto cur_tp = chrono::system_clock::now();
  std::chrono::duration<double> time_diff = cur_tp - prev_tp;
  prev_tp = cur_tp;

  p_error_ = cte;
  d_error_ = (cte - prev_cte_) / time_diff.count();
  prev_cte_ = cte;
  i_error_ += cte * time_diff.count();

  double steering = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;

  /*printf("p_err=%f d_err=%f i_err=%f time_diff=%f tot=%f\n",
         p_error_ * Kp_,
         d_error_ * Kd_,
         i_error_ * Ki_,
         time_diff.count(),
         steering);*/

  steering = (steering < -1.0) ? -1.0 : steering;
  steering = (steering > 1.0) ? 1.0 : steering;

  total_error_ += fabs(cte);

  if (steps++ == TWIDDLE_ITERATIONS)
  {
    twiddle(total_error_);
    steps = 0;
    total_error_ = 0;
  }

  return steering;
}

double PID::TotalError()
{
  return 0;
}

void PID::twiddle(double err)
{
  static int i = 0;
  static int state = 1;

  if (d_[0] + d_[1] + d_[2] < 0.00001)
  {
    return;
  }

  switch (state)
  {
    case 1:
      if (err < best_error_)
      {
        best_error_ = err;
        d_[i] *= 1.1;
        i = (i + 1) % 3;
        p_[i] += d_[i];
        printf("Success.\n");
      }
      else
      {
        p_[i] -= 2 * d_[i];
        state = 2;
      }
      break;
    case 2:
      if (err < best_error_)
      {
        best_error_ = err;
        d_[i] *= 1.1;
        printf("Success.\n");
      }
      else
      {
        p_[i] += d_[i];
        d_[i] *= 0.9;
      }
      state = 1;
      i = (i + 1) % 3;
      p_[i] += d_[i];
      break;
  };

  Kp_ = p_[0];
  Ki_ = p_[1];
  Kd_ = p_[2];

  printf ("Try values: p = %f i = %f d = %f\n", Kp_, Ki_, Kd_);
}
