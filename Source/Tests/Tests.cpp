#include <gtest/gtest.h>
#include <iostream>
#include "Eigen/Core"
#include "../IGRF/igrf.h"

using namespace std;

TEST(ADCS_test, DemonstrateGTestMacros) {
    Eigen::MatrixXd mag_reference(3, 1);
    mag_reference << 0, 0, 0;
    Eigen::MatrixXd& mag = mag_reference;
    uint8_t i = MagReference(-30.56120405, -9.857491524, 577, 2020, 11, 17, mag);
    cout << "Error =" << i << '\n';
    cout << mag_reference(0, 0) << mag_reference(1, 0) << mag_reference(2, 0);
    EXPECT_EQ(i, 0);
}