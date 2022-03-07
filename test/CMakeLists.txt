add_executable(test_init test_init.cpp)
add_executable(test_decompose_H test_decompose_H.cpp)
target_link_libraries(test_init sslam fmt::fmt)
target_link_libraries(test_decompose_H sslam fmt::fmt)