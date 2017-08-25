cmake_minimum_required(VERSION 2.8)

project(joint_pointcloud)

set(CMAKE_CXX_COMPLIER "g++")
set(CMAKE_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PORJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)


