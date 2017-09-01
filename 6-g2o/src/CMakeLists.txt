# Subdirectory src

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
# CSparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})


# user lib
add_library(slam_base SHARED slam_base.cpp)
target_link_libraries(slam_base
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )
# user lib
add_library(viewer_lib SHARED display.cpp)
target_link_libraries(viewer_lib
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

set(G2O_LIBS g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)
# exe
add_executable(g2o g2o.cpp)
target_link_libraries(g2o
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${G2O_LIBS}
    ${CSPARSE_LIBRARY}
    slam_base
    viewer_lib
)



