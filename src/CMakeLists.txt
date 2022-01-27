add_library(sslam SHARED
        config.cpp dataset.cpp feature.cpp frame.cpp front_end.cpp camera.cpp map.cpp map_point.cpp)

target_link_libraries(sslam ${THIRD_PARTY_LIBS})