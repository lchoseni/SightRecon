cmake_minimum_required(VERSION 3.2)
project(sslam)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ")

include_directories("/usr/include/eigen3")
include_directories("/usr/local/include")

link_directories("/usr/local/Cellar/fmt/8.1.1_1/lib")



find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(fmt )


find_package(Eigen3 )
include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/include)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        fmt)

add_subdirectory(src)
add_subdirectory(test)
