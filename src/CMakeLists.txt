add_library(sslam SHARED
        config.cpp dataset.cpp feature.cpp frame.cpp front_end.cpp)

target_link_libraries(sslam)