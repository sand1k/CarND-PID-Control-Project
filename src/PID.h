#ifndef PID_H
#define PID_H

#include <chrono>
using namespace std;

#define TWIDDLE_ITERATIONS 2200

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
  void Init(double Kp, double Ki, double Kd, bool do_twiddle);

  /**
   * Calculate steering angle.
   * @param cte The current cross track error
   * @param speed The current speed
   */
  double CalcSteering(double cte, double speed);

private:
  void twiddle(double err);

private:
  /**
   * PID Varialbes
   */
  double i_error_; // integral error
  double prev_cte_; // cte on previous iteration
  chrono::time_point<chrono::system_clock> prev_tp; // time point of previous cte measurement

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  /**
   * Varialbes to perform twiddle
   */
  bool do_twiddle_;
  double p_[3];
  double d_[3];
  double best_error_;
  double total_error_;

};

#endif  // PID_H
