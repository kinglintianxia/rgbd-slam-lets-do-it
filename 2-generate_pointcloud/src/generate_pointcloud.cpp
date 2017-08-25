#include <iostream>
#include <string>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Point cloud
typedef pcl::PointXYZRGBA PointT;
// Point cloud
pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
// Viewer
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("cloud viewer"));

// Camera intrinsic matrix
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

int main(int argc, char** argv)
{
    cv::Mat color, depth;
    color = cv::imread("../data/rgb.png");
    depth = cv::imread("../data/depth.png",-1);

    for(int v = 0; v < depth.rows; ++v)
    {
        for(int u = 0; u < depth.cols; ++u)
        {
            ushort d = depth.ptr<ushort>(v)[u];
            if(d == 0)
                continue;

            PointT point;
            point.z = (double)d / camera_factor;
            point.x = (u - camera_cx) * point.z / camera_fx;
            point.y = (v - camera_cy) * point.z / camera_fy;

            // Color
            point.b = color.ptr<uchar>(v)[u * 3];
            point.g = color.ptr<uchar>(v)[u * 3 + 1];
            point.r = color.ptr<uchar>(v)[u * 3 + 2];

            cloud->points.push_back(point);

        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->resize(cloud->width * cloud->height);
    cloud->is_dense = false;

    // Save pointcloud
    pcl::io::savePCDFile("../pointcloud.pcd", *cloud);

    // Show pointcloud
    viewer->addPointCloud(cloud);
    // Spin 100ms
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);

    }

    return 0;
}
