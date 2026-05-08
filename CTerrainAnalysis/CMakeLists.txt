add_library(
    CTerrainAnalysisOpen
    STATIC
    CInstantTerrainMapper.cpp
    CAccumedTerrainMapper.cpp
)

target_link_libraries(
    CTerrainAnalysisOpen
    PUBLIC
    MiscOpen
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_include_directories(
    CTerrainAnalysisOpen
    PUBLIC
    ${PROJECT_SOURCE_DIR}
)
