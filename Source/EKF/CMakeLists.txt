#include your files
set(SOURCES Extended_Kalman_Filter_RW.cpp
            Extended_Kalman_Filter_RW.h)
add_library(EKF ${SOURCES})
target_include_directories(EKF PUBLIC /usr/include/eigen3)
#add_definitions("-Wa,-mbig-obj")