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
set(CMAKE_CXX_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_VERSION 9.3.0)

enable_testing()

add_subdirectory(IGRF)
add_subdirectory(Communications)
add_subdirectory(Tests)
add_subdirectory(sunVector)
add_subdirectory(EKF)
add_subdirectory(Nadir)
add_subdirectory(Q_Method)
add_subdirectory(Bdot)
add_subdirectory(PDcontroller)
add_subdirectory(googletest)
add_subdirectory(Sun_Vector_Reference)
add_subdirectory(../JTOK build)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(ADCS main.cpp)

target_link_libraries(ADCS PUBLIC IGRF)
target_link_libraries(ADCS PUBLIC Communications)
target_link_libraries(ADCS PUBLIC SunVector)
target_link_libraries(ADCS PUBLIC Eigen3::Eigen)
target_link_libraries(ADCS PUBLIC EKF)
target_link_libraries(ADCS PUBLIC Nadir)
target_link_libraries(ADCS PUBLIC Q_Method)
target_link_libraries(ADCS PUBLIC Bdot)
target_link_libraries(ADCS PUBLIC PDcontroller)
target_link_libraries(ADCS PUBLIC Sun_Vector_Reference)
target_link_libraries(ADCS PUBLIC JTOK)

target_sources(ADCS PUBLIC ./JSON.cpp)

target_include_directories(ADCS PUBLIC
                            "${PROJECT_BINARY_DIR}"
                            "${PROJECT_SOURCE_DIR}/IGRF"
                            "${PROJECT_SOURCE_DIR}/Communications"
                            "${PROJECT_SOURCE_DIR}/sunVector"
                            "${PROJECT_SOURCE_DIR}/EKF"
                            "${PROJECT_SOURCE_DIR}/Nadir"
                            "${PROJECT_SOURCE_DIR}/Q_Method"
                            "${PROJECT_SOURCE_DIR}/Bdot"
			                "${PROJECT_SOURCE_DIR}/PDcontroller"
                            "${PROJECT_SOURCE_DIR}/Sun_Vector_Reference"
                            "${PROJECT_SOURCE_DIR}/../JTOK"
                            )
