add_executable(test_init test_init.cpp)
add_executable(test_decompose_E test_decompose_E.cpp)
add_executable(test_NCC test_NCC.cpp)
add_executable(test_meanshift test_meanshift.cpp)
add_executable(test_meanshift_colmap test_meanshift_colmap.cpp)


target_link_libraries(test_init sslam fmt::fmt)
target_link_libraries(test_decompose_E  sslam fmt::fmt)
target_link_libraries(test_NCC  sslam fmt::fmt)
target_link_libraries(test_meanshift  sslam fmt::fmt)
target_link_libraries(test_meanshift_colmap  sslam fmt::fmt)

