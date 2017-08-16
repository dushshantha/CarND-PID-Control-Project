#include "PID.h"
#include <iostream>

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
    
    PID::tol = 0.000001;
    
    //Set the below flag to true to run Twiddle
    PID::doTwiddle = false;
    
    PID::p = {0.0, 0.0, 0.0};
    //PID::dp = {1, 1, 1};
    PID::dp = {Kp * 0.01, Ki* 0.008, Kd* 0.005};
    PID::index = 0;
    PID::it = 0;
    
    PID::step1Done = false;
    PID::step2Done = false;
    PID::step3Done = false;
    //std::cout << "step1Done on init : " << step1Done << std::endl;
    
    PID::sum_dp = Kp + Ki + Kd;
    
    
}

void PID::Update(double cte){
    if(doTwiddle && sum_dp > tol){
        // In the first Iteration, Set the cte to best error
        std::cout << "Twiddle starts for param" << index << std::endl;
        if (it == 0){
            
            best_err = cte;
            std::cout << "Iteration 1 Best Error: " << best_err << std::endl;
        }
        // Step 1: Increase the param in index and let the
        // error update on that
        std::cout << "step1Done : " << step1Done << std::endl;
        if(!step1Done){
            std::cout << "Step 1 Done" << std::endl;
            p[index] += dp[index];
            step1Done = true;
        }
        else if(!step2Done){
            if(cte < best_err){
                // got best error for the param in index
                // Moving to the next param
                best_err = cte;
                dp[index] *= 1.1;
                index = (index + 1) % 3;
                

                // Resetting the indicators for the next param
                step1Done = false;
                step2Done = false;
                step3Done = false;
            }
            else {
                // No best error
                // Reverse
                p[index] -= 2 * dp[index];
                step2Done =  true;
                
            }
        }
        else if(!step3Done){
            if(cte < best_err){
                best_err = cte;
                dp[index] *= 1.1;
                
                index = (index + 1) % 3;
                // Resetting the indicators for the next param
                step1Done = false;
                step2Done = false;
                step3Done = false;

            }
            else {
                p[index] += dp[index];
                dp[index] *= 0.9;
                //step3Done = true;
                //step2Done = false;
            }
        }
        
        // Set the new Params for Error Update
        Kp = p[0];
        Ki = p[1];
        Kd = p[2];
        
        sum_dp = dp[0] + dp[1] + dp[2];
        it++;
    }
    
    
    // DEBUG
    std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd <<std::endl;

    UpdateError(cte);
   
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

