//This file runs the tests for the project
#include <gtest/gtest.h>

int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

