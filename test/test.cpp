/**
 * @file        PID.cpp
 * @author      Lidia Al-Zogbi
 * @copyright   3-clause BSD
 * @date        09/27/2019
 * @version     1.0
 *
 * @brief       This file is the cpp file for the PID header file.
 */

#include <vector>
#include <gtest/gtest.h>
#include "PID.h"

/**
 * @brief test function to test if there is error in the code
 * @param PIDTest1 which defined the name of test bench
 * @param properConcatenation is description of the test
 * @return none
 */
TEST(PIDTest1, properConcatenation) {
    std::vector<double> randomVector = {1, 2, 3, 4};
    double newNumber = 5;

    PidController object;
    object.accumulatedErrors(randomVector, newNumber);

    std::vector<double> groundTruth = {1, 2, 3, 4, 5};
    EXPECT_EQ(randomVector, groundTruth);
}


/**
 * @brief test function to test if there is error in the code
 * @param PIDTest1 which defined the name of test bench
 * @param properConcatenation is description of the test
 * @return none
 */
TEST(PIDTest2, computeMethod) {
    PidController object;

    double previousError = 2;
    double currentError = 1;
    double controllerOutput;
    std::vector<double> randomVector = {2, 1};

    controllerOutput = object.computeOutput(previousError, \
        currentError, randomVector);
    double groundTruth = 14;

    EXPECT_EQ(controllerOutput, groundTruth);
}
