#ifndef PID_H
#define PID_H

#include <vector>

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
   * Sum of all CTEs
   */
  double sum_cte;
    
  /*
   * Previous cte
   */
  double prev_cte;
    
  /*
   * Use Twiddle?
   */

  bool doTwiddle;
  bool step1Done;
  bool step2Done;
  bool step3Done;
    
  double tol;
  double best_err;
  double sum_dp;
    
  std::vector<double> p;
  std::vector<double> dp;
    
  int index;
  int it;


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
  void Init(double Kp, double Ki, double Kd);
    
  /*
   * Run the needed functions to update the error
   */
  void Update(double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
