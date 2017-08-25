# Subdirectory src

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_library(slam_base SHARED slam_base.cpp)
target_link_libraries(slam_base ${${OpenCV_LIBS}}
    ${PCL_LIBRARIES}
    )


add_executable(joint_pointcloud joint_pointcloud.cpp)
target_link_libraries(joint_pointcloud
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    slam_base
)



