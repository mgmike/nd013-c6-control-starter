/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(vector<double> Ki, double output_lim_maxi, double output_lim_mini){
  K = Ki;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
}
void PID::Init(vector<double> Ki, vector<double> Di, double output_lim_maxi, double output_lim_mini){
  D = Di;
  Kn = Di;
  PID::Init(Ki, output_lim_maxi, output_lim_mini);
}

void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/

  if (abs(delta_time) <= 0.00001) return;
  cte_p = cte;
  cte_d = (cte - cte_prev) / delta_time;
  cte_i += cte * delta_time;
  cte_prev = cte;
  error = - K[0] * cte_p - K[1] * cte_i - K[2] * cte_d;
}

double PID::TotalError() {
   /**
   * Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control;

   control = error;
   total_err += pow(error, 2);
   if (control < output_lim_min) control = output_lim_min;
   else if (control > output_lim_max) control = output_lim_max;
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}