# Subdirectory src
set(OpenCV_DIR "/home/king/Documents/opencv-3.4.0/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_library(slam_base SHARED slam_base.cpp)
target_link_libraries(slam_base
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

add_library(viewer_lib SHARED display.cpp)
target_link_libraries(viewer_lib
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )


add_executable(vo vo.cpp)
target_link_libraries(vo
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    slam_base
    viewer_lib
)



