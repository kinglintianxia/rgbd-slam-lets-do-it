# subdirectory src
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})



add_executable(genetate_pointcloud generate_pointcloud.cpp)
target_link_libraries(genetate_pointcloud ${PCL_LIBRARIES}
                                          ${OpenCV_LIBS}
                     )

