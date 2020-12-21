#include your files
set(SOURCES Sun_Vector_Estimate.cpp
            Sun_Vector_Estimate.h)
add_library(SunVector ${SOURCES})
target_include_directories(SunVector PUBLIC /usr/include/eigen3)
