#include "PID.h"
#include <stdio.h>

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
  d_[0] = 0.1;
  d_[1] = 0.1;
  d_[2] = 0.1;
  best_error_ = 0.0;
}

double PID::UpdateError(double cte)
{
  auto cur_tp = chrono::system_clock::now();
  std::chrono::duration<double> time_diff = cur_tp - prev_tp;
  prev_tp = cur_tp;

  p_error_ = cte;
  d_error_ = (cte - prev_cte_) / time_diff.count();
  prev_cte_ = cte;
  i_error_ += cte * time_diff.count();

  double t_error = -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;

  printf("p_err=%f d_err=%f i_err=%f time_diff=%f tot=%f\n",
         p_error_ * Kp_,
         d_error_ * Kd_,
         i_error_ * Ki_,
         time_diff.count(),
         t_error);

  t_error = (t_error < -1.0) ? -1.0 : t_error;
  t_error = (t_error > 1.0) ? 1.0 : t_error;

  return t_error;
}

double PID::TotalError()
{
  return 0;
}

void PID::twiddle(double err)
{
  static int iter = 0;
  static int state = -1; // init
  static bool is_increase = true;

  if (d_[0] + d_[1] + d_[2] < 0.00001)
  {
    return;
  }

  if (state == -1)
  {
    best_error_ = err;
    state = 0;
    return;
  }

  if (err < best_error_)
  {
    best_error_ = err;
    d_[iter] = is_increase ? 1.1 : 0.9;
  }





  p_[iter] += d[iter];
    double err = 0;
    if (err < best_error_)
    {
      best_error_ = err;
      d_[i] *= 1.1;
    }
    else {
      p_[i] -= 2 * d_[i];
      
      err = 
      if (err < best_error_)
      {
        best_error_ = err;
        d_[i] *= 1.1;
      }
      else
      {
        p_[i] += d_[i];
        d_[i] *= 0.9;
    }

  }

}
