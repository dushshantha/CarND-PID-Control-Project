#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    
    PID::p_error = 0.0;
    PID::i_error = 0.0;
    PID::d_error = 0.0;
    
    PID::sum_cte = 0.0;
    PID::prev_cte = 1.0;
    
    
    
}

void PID::UpdateError(double cte) {
    sum_cte += cte;
    p_error = cte;
    i_error = sum_cte;
    d_error = cte - prev_cte;
    prev_cte = cte;
    
}

double PID::TotalError() {
    return -(Kp * p_error + Ki * i_error + Kd * d_error);
}

