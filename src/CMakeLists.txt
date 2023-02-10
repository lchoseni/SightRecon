add_library(srecon SHARED
        config.cpp 
        dataset/image_dataset.cpp dataset/ktti_dataset.cpp        dataset/fountain_dataset.cpp dataset/euroc_dataset.cpp

        feature.cpp 
        frame.cpp 
        camera.cpp 
        map.cpp 
        map_point.cpp 
        graph.cpp 
        common_include.cpp
        hmm.cpp 
        utils/utils.cpp 
        feature_track.cpp 
        feature/extractor.cpp feature/orb_extractor.cpp parameter.cpp 
        initial/mono_initial.cpp
        mono_track.cpp
        optimizer.cpp
        mono_sys.cpp
        viewer.cpp
        imu/imu.cpp
        processor/image_processor.cpp
        data_publish.cpp
        )

# target_link_libraries(srecon ${THIRD_PARTY_LIBS} pango_display pango_python)
target_link_libraries(srecon ${THIRD_PARTY_LIBS} )