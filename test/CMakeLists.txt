add_executable(test_init test_init.cpp)
add_executable(test_decompose_E test_decompose_E.cpp)
target_link_libraries(test_init sslam fmt::fmt)
target_link_libraries(test_decompose_E sslam fmt::fmt)