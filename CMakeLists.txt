cmake_minimum_required(VERSION 3.2)
project(sslam)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall ")

include_directories("/usr/include/eigen3")
include_directories("/usr/local/include")

link_directories("/usr/local/Cellar/fmt/8.1.1_1/lib")
#link_directories(" /usr/local/lib")
 link_libraries("/usr/local/lib/libfmt.a")



find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(fmt REQUIRED)
message("fmt ${fmt_FOUND}")

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} )

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message("Sophus ${Sophus_INCLUDE_DIRS} ${Sophus_LIBRARIES}")

find_package(Eigen3 )
include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/include)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
#        Sophus::Sophus
#        ${Sophus_INCLUDE_DIRS}
        ${CERES_LIBRARIES}
        ${Boost_LIBRARIES}
        opencv_cvv
        opencv_xfeatures2d
        )

add_subdirectory(src)
add_subdirectory(test)
