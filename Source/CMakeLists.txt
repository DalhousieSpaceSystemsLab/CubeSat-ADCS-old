#this is the main cmake file that builds the system
#Sets minimum version of cmake
cmake_minimum_required(VERSION 3.0)

#set project name and version
project(CubeSat-ADCS VERSION 1.0)

#configure version file
configure_file(CubeSat_ADCSConfig.h.in CubeSat_ADCSConfig.h)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(IGRF)
add_subdirectory(Communications)
#add_subdirectory(Test)
add_subdirectory(sunVector)

add_executable(ADCS main.cpp)

target_link_libraries(ADCS PUBLIC IGRF)
target_link_libraries(ADCS PUBLIC Communications)
target_link_libraries(ADCS PUBLIC SunVector)

target_include_directories(ADCS PUBLIC
			    /usr/include/eigen3
                            "${PROJECT_BINARY_DIR}"
                            "${PROJECT_SOURCE_DIR}/IGRF"
                            "${PROJECT_SOURCE_DIR}/Communications"
                            "${PROJECT_SOURCE_DIR}/sunVector"
                            )
