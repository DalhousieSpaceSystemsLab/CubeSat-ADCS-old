#include your files
set(SOURCES sun_vector_reference.cpp
            sun_vector_reference.h)
add_library(Sun_Vector_Reference ${SOURCES})
target_include_directories(Sun_Vector_Reference PUBLIC /usr/include/eigen3)