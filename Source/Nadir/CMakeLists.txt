#include your files
set(SOURCES Nadir.cpp
            Nadir.h)
add_library(Nadir ${SOURCES})
target_include_directories(Nadir PUBLIC /usr/include/eigen3)