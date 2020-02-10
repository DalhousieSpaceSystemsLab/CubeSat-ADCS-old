#set the sources for this library
set(SOURCES 
    igrf_parse.cpp
    igrf12.hpp
    igrf12.cpp
    legendre.cpp
    IGRF12.txt)
#this adds the module to the project
add_library(IGRF12 ${SOURCES})