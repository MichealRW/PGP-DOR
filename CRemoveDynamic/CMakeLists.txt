add_library(
    CRemoveDynamicOpen
    STATIC
    CRemoveDynamicV2.cpp
    DynamicMap.cpp
)

target_link_libraries(
    CRemoveDynamicOpen
    PUBLIC
    CTerrainAnalysisOpen
    MiscOpen
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_include_directories(
    CRemoveDynamicOpen
    PUBLIC
    ${PROJECT_SOURCE_DIR}
)
