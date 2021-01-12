#include <gtest/gtest.h>
#include <iostream>
#include "../IGRF/igrf.h"

using namespace std;

TEST(ADCS_test, DemonstrateGTestMacros) {
    uint8_t i = IGRF(30.166923849507349, 23, 300, 2021, 12, 31);
    EXPECT_EQ(i, 1);
}