# CubeSat ADCS
## Overview:
This repository contains the attitude determination and control subsystem (ADCS) software for the LORIS CubeSat. Orbital and attitude dynamics are propagated onboard from an initial GPS state. The satellite attitude is controlled based on coarse sensor readings and onboard model predictions. Three operational phases are considered:
- The launch and early operations phase (LEOP) begins 30 minutes post-launch and transitions into detumbling the satellite at its earliest convenience
- The nominal operations (pointing) phase points the +Z (camera) face nadir within ±5° accuracy
- The low-power safe mode powers down all but essential components of the ADCS
## Objectives:
- Parse GPS data for use in predicting the satellite's orbital position and velocity
- Model necessary orbital and attitude dynamics onboard the satellite
- Model sun and magnetic field reference vectors onboard the satellite
- Use typical proportional B-dot control to detumble the satellite
- Use PD control to point the satellite within the desired degree of accuracy
- Handle transitions between the distinct ADCS operational phases
## Language:
- C++ 11
## Target Operating System:
- ...
## Dependencies:
- [CMake](https://cmake.org/)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - C++ library for vector and matrix operations
  - Add to your system library by runnning _sudo apt install libeigen3-dev_
- [Google Test](https://github.com/google/googletest/)
  - Library used for running tests with cmake during build.
  - Add to your system library on linux by running 
  - _sudo apt-get install libgtest-dev_ 
  - _cd usr/src/gtest_
  - _sudo cmake CMakeLists.txt_
  - _sudo make_
  - _sudo cp *.a /usr/lib_
- [Dalhousie LORIS CubeSat GPS software](https://github.com/DalhousieSpaceSystemsLab/CubeSat-GPS)
  - To set initial states for and periodically update the onboard orbit, environment, and attitude models 
