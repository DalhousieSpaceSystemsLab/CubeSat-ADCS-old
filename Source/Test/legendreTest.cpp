//This file tests the legendre class for our project
#include <gtest/gtest.h>
#include "igrf12.hpp"

TEST(igrf12_Test, Test1){
    //prepare legendre function
    legendre testLegendre(4,4,0.707,0.707);
    
    //add the conditions
    EXPECT_EQ (testLegendre.dpMNBar, 0);
    EXPECT_EQ (testLegendre.pMNBar,0);
}