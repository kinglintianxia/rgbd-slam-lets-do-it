#include "slam_base.h"
// stdlib
#include <iostream>
#include <string>
#include <fstream>
//OpenCV
#include <opencv2/core/eigen.hpp>
// PCL
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    ParameterReader filereader("../parameters.txt");

    // Frame
    Frame frame1, frame2;
    frame1.color = cv::imread("../data/rgb1.png");
    frame2.color = cv::imread("../data/rgb2.png");
    frame1.depth = cv::imread("../data/depth1.png", -1);
    frame2.depth = cv::imread("../data/depth2.png", -1);

    // Cmpute descritors and find matches
    compute_keypoints_descriptors(frame1, filereader.get_data("detector").c_str(), filereader.get_data("descriptor").c_str());
    compute_keypoints_descriptors(frame2, filereader.get_data("detector").c_str(), filereader.get_data("descriptor").c_str());

    // Camera parameters
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = std::atof(filereader.get_data("camera.fx").c_str());
    camera.fy = std::atof(filereader.get_data("camera.fy").c_str());
    camera.cx = std::atof(filereader.get_data("camera.cx").c_str());
    camera.cy = std::atof(filereader.get_data("camera.cy").c_str());
    camera.scale = std::atof(filereader.get_data("camera.scale").c_str());

    // Estimate motion
    Result_of_pnp result;
    result = estimate_motion(frame1, frame2, camera);

    // Image to pointcloud
    pointcloud::Ptr cloud1 (new pointcloud()), cloud2 (new pointcloud()), cloud1_trans (new pointcloud());
    cloud1 = image_to_pointcloud(frame1, camera);
    cloud2 = image_to_pointcloud(frame2, camera);
    std::cout <<"cloud1: " << cloud1->points.size() << std::endl;
    // Transform cloud
    /*
     *   transformPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                       pcl::PointCloud<PointT> &cloud_out,
                       const Eigen::Vector3f &offset,
                       const Eigen::Quaternionf &rotation)
     *
     */
    // Data prepare
    Eigen::Vector3f offset;
//////////// Only at<double> works!!!
    offset << result.tvect.at<double>(0,0), result.tvect.at<double>(0,1), result.tvect.at<double>(0,2);
    std::cout << "offset: \n" << offset << std::endl;
    // Rotation
    cv::Mat r_mat;
    cv::Rodrigues(result.rvect, r_mat);
    Eigen::Matrix3f r_eigen;
    cv::cv2eigen(r_mat, r_eigen);
    Eigen::Quaternionf rotation(r_eigen);

    // Transform pointcloud
    pcl::transformPointCloud(*cloud1, *cloud1_trans, offset, rotation);
    *cloud1_trans += *cloud2;
    // Write pointcloud
    pcl::PCDWriter writer;
    writer.write("../data/joint_pointcloud.pcd", *cloud1_trans);

    // Show pointcloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("joint pointcloud"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.5);
    viewer->addPointCloud(cloud1_trans, "joint cloud", 0);

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);

    }


    return 0;
}




