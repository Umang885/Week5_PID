/**
 * @file        PID.cpp
 * @author      Lidia Al-Zogbi
 * @copyright   3-clause BSD
 * @date        09/27/2019
 * @version     1.0
 *
 * @brief       This file is the cpp file for the PID header file.
 */

#include "../include/PID.hpp"
#include <iostream>
#include <numeric>

/**
 * @brief       PIDController class
 */

double PIDController::ComputeError(double& SetPoint, double& PlantOutput){
    double CurrentError = SetPoint - PlantOutput;
    return CurrentError;
}

double PIDController::ComputeOutput(double& PreviousError, double& CurrentError, std::vector<double>& AccumulatedErrors){
    
//    double PropTerm = this->Kp*CurrentError;
//    double DerTerm = this->Kd*(CurrentError - PreviousError);
//    double IntTerm = this->Ki*(accumulate(AccumulatedErrors.begin(),AccumulatedErrors.end(),0));
//    double TotalOutput = PropTerm + DerTerm + IntTerm;
//
//    return TotalOutput;
    
    return 5;
}

std::vector<double> PIDController::AccumulatedErrors(std::vector<double>& AccumulatedErrors, double& CurrentError){
        AccumulatedErrors.push_back(CurrentError);
	std::vector<double> Any = {23, 53, 64};
        return Any;
    }
