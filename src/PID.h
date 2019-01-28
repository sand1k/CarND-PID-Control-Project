#ifndef PID_H
#define PID_H

#include <chrono>
using namespace std;

class PID
{
public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

private:
  void twiddle();

private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  double p_[3];
  double d_[3];
  double t_error_;

  double prev_cte_; // cte on previous iteration
  chrono::time_point<chrono::system_clock> prev_tp;
};

#endif  // PID_H
