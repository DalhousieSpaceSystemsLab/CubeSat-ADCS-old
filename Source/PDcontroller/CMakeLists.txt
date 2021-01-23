#include your files
set(SOURCES PDcontroller.cpp
            PDcontroller.h)
add_library(PDcontroller ${SOURCES})
target_include_directories(PDcontroller PUBLIC /usr/include/eigen3)