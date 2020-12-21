#include your files
set(SOURCES Q_Method.cpp
            Q_Method.h)
add_library(Q_Method ${SOURCES})
target_include_directories(Q_Method PUBLIC /usr/include/eigen3)