cmake_minimum_required(VERSION 2.8)

project(octomap)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INLCUDE_DIRS})
# Eigen
include_directories("/usr/include/eigen3")

##################################
# Octomap
add_executable(octomap src/pcd_to_octomap.cpp)
target_link_libraries(octomap
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    )

# Color Octomap
add_executable(color_octomap src/pcd_to_color_octomap.cpp)
target_link_libraries(color_octomap
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    )

# joint octomap
add_executable(joint_octomap src/joint_octomap.cpp)
target_link_libraries(joint_octomap
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )
