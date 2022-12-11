cmake_minimum_required(VERSION 3.2)
project(sightRecon)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall ")

include_directories("/usr/include/eigen3")
include_directories("/usr/local/include")

link_directories("/usr/local/Cellar/fmt/8.1.1_1/lib")

################################
# GTest
################################
find_package(GTest REQUIRED)
enable_testing()
include_directories(${GTEST_INCLUDE_DIRS})

find_package(Threads REQUIRED)

find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(fmt )
find_package(Ceres REQUIRED)

find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Eigen3 )
include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/include)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        )

add_subdirectory(src)
add_subdirectory(test)
