add_library(sslam SHARED
        config.cpp dataset.cpp feature.cpp frame.cpp front_end.cpp camera.cpp map.cpp map_point.cpp graph.cpp common_include.cpp
        hmm.cpp utils/utils.cpp BA.cpp SFM.cpp mean_shift.cpp fusion.cpp ColmapDataset.cpp ../include/SSLAM/ColmapDataset.h)

target_link_libraries(sslam ${THIRD_PARTY_LIBS} )
