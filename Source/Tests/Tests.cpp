#include <gtest/gtest.h>
#include <iostream>
#include "Eigen/Core"
#include "../IGRF/igrf.h"

using namespace std;

TEST(ADCS_test, DemonstrateGTestMacros) {
    const double ACCURACY = 0.0000000001;   //  1e-10
    Eigen::MatrixXd mag_reference(3, 1);
    mag_reference << 0, 0, 0;       //  Initialize matrix
    Eigen::MatrixXd& mag = mag_reference;

    // Expected values -
    // 09132.18030132573  -03431.07004676884  -17758.37773377087
    uint8_t i = MagReference(-30.56120405, -9.857491524, 577, 2020, 11, 17, mag);
    cout << (int)i << '\n';
    cout << mag_reference(0, 0) << mag_reference(1, 0) << mag_reference(2, 0);

    //  Check for return value
    EXPECT_EQ(i, 0);

    //  Test for accuracy
    EXPECT_NEAR(9132.18030132573, mag_reference(0, 0), ACCURACY);
    EXPECT_NEAR(-3431.07004676884, mag_reference(1, 0), ACCURACY);
    EXPECT_NEAR(-17758.37773377087, mag_reference(2, 0), ACCURACY);
}