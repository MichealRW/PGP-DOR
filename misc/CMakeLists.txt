add_library(
    MiscOpen
    STATIC
    BGK_Smoother.cpp
    Colors.cpp
    CommenDefinition.cpp
    CoordTransformation.cpp
    FH_kdtree.cpp
    Utils.cpp
    Visualize.cpp
)

target_link_libraries(
    MiscOpen
    PUBLIC
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_include_directories(
    MiscOpen
    PUBLIC
    ${PROJECT_SOURCE_DIR}
)
