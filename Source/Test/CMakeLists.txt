#use this file to create unit tests to test individual methods and modules
#we can write tests that will become part of the build process and fail the buil if a test is failed
#in order for this to work you need to get googletest as part of your system library read the README.md for more info

#sets the binary directory for the tests
set(BINARY ${CMAKE_PROJECT_NAME}_Test)

#enable testing
enable_testing()

#make all files visible
file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.hpp *.cpp)
set(SOURCES ${TEST_SOURCES})

add_executable(${BINARY} ${TEST_SOURCES})
add_test(NAME ${BINARY}, COMMAND ${BINARY})

target_link_libraries(${BINARY} PUBLIC IGRF12)

target_include_directories(${BINARY} PUBLIC "${PROJECT_SOURCE_DIR}/IGRF12")