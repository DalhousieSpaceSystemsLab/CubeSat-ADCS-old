cmake_minimum_required(VERSION 3.12.2)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/../CubeSat-ADCS/Build)
set (CMAKE_BINARY_DIR ${CMAKE_HOME_DIRECTORY}/../CubeSat-ADCS)

project(CubeSat-ADCS)	#project name

include_directories (
    Eigen			# includes Eigen which is located inside the project folder
	)
project(MatrixTesting)

include_directories (
	Eigen
)
add_executable ( MatrixTesting 
    main.cpp
)
