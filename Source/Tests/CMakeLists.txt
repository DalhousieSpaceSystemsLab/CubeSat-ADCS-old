cmake_minimum_required(VERSION 3.0)

set(This ADCS_test)

set(Sources Tests.cpp)

add_executable(${This} ${Sources})

target_link_libraries(${This} PUBLIC
    gtest_main
    IGRF
    Sun_Vector_Reference
    SunVector)

target_include_directories(${This} PUBLIC 
    /usr/include/eigen3)

add_test(
    NAME ${This}
    COMMAND ${This}
)

add_test(
    NAME Hello
    COMMAND ${This}
)