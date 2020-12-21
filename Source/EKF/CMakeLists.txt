#include your files
set(SOURCES Extended_Kalman_filter_RW.cpp
            Extended_Kalman_filter_RW.h)
add_library(EKF ${SOURCES})
target_include_directories(EKF PUBLIC /usr/include/eigen3)