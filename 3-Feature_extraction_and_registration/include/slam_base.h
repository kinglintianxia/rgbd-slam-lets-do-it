# pragma once

#include <iostream>
#include <fstream>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> pointcloud;

// Camera intrinsic
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;

};

pointcloud::Ptr image_to_pointcloud(cv::Mat rgb, cv::Mat depth, CAMERA_INTRINSIC_PARAMETERS camera);

cv::Point3f point2d_to_3d(cv::Point3f point, CAMERA_INTRINSIC_PARAMETERS camera);


