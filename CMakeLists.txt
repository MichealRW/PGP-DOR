cmake_minimum_required(VERSION 3.16)

project(PGP_DOR_Open)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-Wall -Wextra -O3)
else()
    add_compile_options(-Wall -Wextra -Wno-return-type -Wno-deprecated-declarations)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters search registration octree visualization)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs pcl_conversions)

include_directories(
    .
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(misc)
add_subdirectory(CTerrainAnalysis)
add_subdirectory(CRemoveDynamic)

add_executable(pgp_dor_kitti main/pgp_dor_kitti.cpp)
target_link_libraries(
    pgp_dor_kitti
    PRIVATE
    CRemoveDynamicOpen
    CTerrainAnalysisOpen
    MiscOpen
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(pgp_dor_ros_node main/pgp_dor_ros_node.cpp)
target_link_libraries(
    pgp_dor_ros_node
    PRIVATE
    CRemoveDynamicOpen
    CTerrainAnalysisOpen
    MiscOpen
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)
