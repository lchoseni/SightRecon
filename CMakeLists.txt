cmake_minimum_required(VERSION 2.8)
project(sslam)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")


include_directories("/usr/include/eigen3")

find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(fmt)
#target_link_libraries(sslam fmt::fmt)

include_directories(${PROJECT_SOURCE_DIR}/include)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS})

add_subdirectory(src)
add_subdirectory(test)
