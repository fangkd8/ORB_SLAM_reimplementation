add_library( VisualOdometry SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    VisualOdometry.cpp
    g2o_type.cpp
)

target_link_libraries( VisualOdometry
    ${THIRD_PARTY_LIBS}
)
