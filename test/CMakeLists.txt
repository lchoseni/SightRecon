add_executable(test_init test_init.cpp)
add_executable(test_decompose_E test_decompose_E.cpp)
add_executable(test_NCC test_NCC.cpp)


target_link_libraries(test_init sslam fmt)
target_link_libraries(test_decompose_E fmt sslam )
target_link_libraries(test_NCC fmt sslam )

