#set the sources for this library
set(SOURCES 
    igrf.cpp
    igrf.h
    IGRF13_COEFF.h)
#this adds the module to the project
add_library(IGRF ${SOURCES})
