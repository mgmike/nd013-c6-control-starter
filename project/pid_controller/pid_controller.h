/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <vector>
#include <limits>

class PID {
public:

   /**
   * Create the PID class
   **/

    /*
    * Errors
    */

   double cte_p = 0.0;
   double cte_prev = 0.0;
   double cte_d = 0.0;
   double cte_i = 0.0;
   double error = 0.0;
   double best_err = std::numeric_limits<double>::max();
   double total_err = 0.0;

    /*
    * Coefficients
    */

   std::vector<double> K;
   std::vector<double> D;

    /*
    * Output limits
    */

   double output_lim_max;
   double output_lim_min;

    /*
    * Twiddle variables
    */

   int position[3] = {1,1,1};
   std::vector<double> Kn;
   int n = 0;
  
    /*
    * Delta time
    */

   double delta_time = 0.0;

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
    void Init(std::vector<double> K, double output_lim_max, double output_lim_min);
    void Init(std::vector<double> K, std::vector<double> D, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


