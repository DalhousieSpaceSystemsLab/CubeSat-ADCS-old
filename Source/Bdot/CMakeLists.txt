#include your files
set(SOURCES Bdot.cpp
            Bdot.h)
add_library(Bdot ${SOURCES})
target_include_directories(Bdot PUBLIC /usr/include/eigen3)