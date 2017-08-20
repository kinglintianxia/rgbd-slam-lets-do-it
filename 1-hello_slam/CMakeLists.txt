cmake_minimum_required(VERSION 2.8)

project(hello_slam)

set(CMAKE_CXX_COMPLIER "g++")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
link_directories(${PORJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)

