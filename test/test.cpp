/**
 * @file        PID.cpp
 * @author      Lidia Al-Zogbi
 * @copyright   3-clause BSD
 * @date        09/27/2019
 * @version     1.0
 *
 * @brief       This file is the cpp file for the PID header file.
 */

#include "../app/PID.cpp"
#include <gtest/gtest.h>

TEST(PIDTest1, ProperConcatenation){
    std::vector<double> RandomVector = {1, 2, 3, 4};
    double NewNumber = 5;
    std::vector<double> NewVector;
    
    PIDController Object;
    NewVector = Object.AccumulatedErrors(RandomVector, NewNumber);
    
    std::vector<double> GroundTruth = {1, 2, 3, 4, 5};
    EXPECT_EQ(NewVector, GroundTruth);
}

TEST(PIDTest2, ComputeMethod){
    PIDController Object;
    
    double PreviousError = 2;
    double CurrentError = 1;
    double ControllerOutput;
    std::vector<double> RandomVector = {1, 2, 3, 4};
    
    ControllerOutput = Object.ComputeOutput(PreviousError, CurrentError, RandomVector);
    
    double GroundTruth = 1;
    
    EXPECT_EQ(ControllerOutput, GroundTruth);
}
