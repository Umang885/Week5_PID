/**
 * @file        PID.hpp
 * @author      Lidia Al-Zogbi
 * @copyright   3-clause BSD
 * @date        09/27/2019
 * @version     1.0
 *
 * @brief       This file is the header file for the PID header file
 */

#ifndef _PID_H_
#define _PID_H_
#include <vector>

/**
 * @brief       PIDController class
 */

class PIDController {
private:
    /**
     * @brief Defining parameters for the controller, namely proportional gain
     * (Kp), derivative gain (Kd) and integral gain (Ki)
     */
    double Kp;
    double Kd;
    double Ki;
    
public:
    /**
     * @brief class constructor defining parameter values
     */
    
    PIDController(){
        Kp = 10;
        Kd = 5;
        Ki = 3;
    }
    
    double SetPoint = 0;
    double PlantOutput = 0;
    double PIDOutput = 0;
    
    /**
     * @brief Function computing the error between the setpoint and
     * plant output to form the input to the controller.
     * @param SetPoint and plantoutput
     * @return computes error
     */
    double ComputeError(double& SetPoint, double& PlantOutput);
    
    /**
     * @brief Function for concatenating errors in a vector to approximate
     * the integral
     * @param vector of previous errors and most recent computed error
     * @return vector of previous error with added new error
     */
    std::vector<double> AccumulatedErrors(std::vector<double>& AccumulatedErrors, double& CurrentError);
    
    /**
     * @brief Function for computing the output velocity of the controller
     * @param current error and vector of
     * accumulated errors
     * @return control signal as an output from the plant
     */
    double ComputeOutput(double& PreviousError, double& CurrentError, std::vector<double>& AccumulatedErrors);
};

#endif
