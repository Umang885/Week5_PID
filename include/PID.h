/**
 * @file        PID.hpp
 * @author      Lidia Al-Zogbi
 * @copyright   3-clause BSD
 * @date        09/27/2019
 * @version     1.0
 *
 * @brief       This file is the header file for the PID header file
 */

#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>
#include <numeric>

/**
 * @brief       PidController class
 */

class PidController {
  private:
    /**
     * @brief Defining parameters for the controller, namely proportional gain
     * (kp), derivative gain (kd) and integral gain (ki)
     */
    double kp;
    double kd;
    double ki;

  public:
    /**
     * @brief class constructor defining parameter values
     */
    PidController();

    double setPoint = 0;
    double plantOutput = 0;
    double pidOutput = 0;

    /**
     * @brief Function computing the error between the setPoint and
     * plant output to form the input to the controller.
     * @param setPoint and plantOutput
     * @return computes error
     */
    double computeError(double& setPoint, double& plantOutput);

    /**
     * @brief Function for concatenating errors in a vector to approximate
     * the integral
     * @param vector of previous errors and most recent computed error
     * @return void
     */
    void accumulatedErrors(std::vector<double>& accumulatedError, double& currentError);

    /**
     * @brief Function for computing the output velocity of the controller
     * @param current error and vector of
     * accumulated errors
     * @return control signal as an output from the plant
     */
    double computeOutput(double& previousError, double& currentError, std::vector<double>& accumulatedError);
};

PidController::PidController() {
  kp = 10;
  kd = 5;
  ki = 3;
}

double PidController::computeError (double& setPoint, double& plantOutput) {
  double currentError = setPoint - plantOutput;
  return currentError;
}

double PidController::computeOutput (double& previousError, double& currentError, std::vector<double>& accumulatedError) {
  double propTerm = this->kp * currentError;
  double derTerm = this->kd * (currentError - previousError);
  double intTerm = this->ki * (accumulate(accumulatedError.begin(), accumulatedError.end(), 0));
  double totalOutput = propTerm + derTerm + intTerm;

  return totalOutput;
}

void PidController::accumulatedErrors (std::vector<double>& accumulatedError, double& currentError) {
  accumulatedError.push_back(currentError);
  // return accumulatedError;
}

#endif  // PID_H
