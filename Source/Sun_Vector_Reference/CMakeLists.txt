#include your files
set(SOURCES sun_vector_reference.cpp
            sun_vector_reference.h)
add_library(SunVecRef ${SOURCES})
target_include_directories(SunVecRef PUBLIC /usr/include/eigen3)