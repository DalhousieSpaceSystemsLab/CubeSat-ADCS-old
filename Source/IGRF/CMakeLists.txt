#set the sources for this library
set(SOURCES 
    igrf.cpp
    igrf.hpp
    IGRF12.txt)
#this adds the module to the project
add_library(IGRF12 ${SOURCES})