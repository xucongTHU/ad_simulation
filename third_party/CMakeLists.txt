cmake_minimum_required(VERSION 3.20.5)

project(third_party)

set(CMAKE_CXX_FLAGS "-std=c++17 -fext-numeric-literals -mavx2 -mno-avx512f -pipe -O2 -Wall -Wextra -fopenmp -fPIC -pthread")

set(PROJECT_INCLUDE_DIRS .)

find_package(catkin REQUIRED COMPONENTS
             roscpp
)

catkin_package(INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
               LIBRARIES
        smartsim
)

include(smartsim_gen.cmake)

include_directories(${PROJECT_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)

install(DIRECTORY .
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY osqp/lib
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "*.a" PATTERN "*.so"
)

add_subdirectory(examples)
