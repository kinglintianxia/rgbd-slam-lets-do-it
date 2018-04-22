#include "slam_base.h"
#include "display.h"
// stdlib
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

//OpenCV
#include <opencv2/core/eigen.hpp>
// PCL
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

void read_frame(ParameterReader read, int cnt, Frame& frame)
{
    std::stringstream ss;
    // rgb
    ss << read.get_data("rgb_dir") + std::to_string(cnt) + read.get_data("rgb_extension");
    frame.color = cv::imread(ss.str());
    // depth
    //!!!!!!! ss.clear() can not clear string just the state of ss.
    ss.str("");
    ss << read.get_data("depth_dir") + std::to_string(cnt) + read.get_data("depth_extension");
    // !!!!!!!!!!!!!!!  -1 is neccesary!!!
    frame.depth = cv::imread(ss.str(), -1);
    std::cout << "Read current frame: " << ss.str() << std::endl;
    if (frame.color.empty() || frame.depth.empty())
    {
        std::cout << "Read frame error, please check out!" << std::endl;
        exit(1);
    }
}

double norm_of_transform(Result_of_pnp result)
{
    return fabs(std::min(cv::norm(result.rvect), (2 * M_PI - cv::norm(result.rvect))) + fabs(cv::norm(result.tvect)));

}


int main(int argc, char** argv)
{
    // Get parameter
    ParameterReader para_read("/home/king/Documents/king/rgbd-slam-lets-do-it/5-VO/parameters.txt");
    // Camera
    CAMERA_INTRINSIC_PARAMETERS camera;
    get_cam_param(para_read, camera);
    // Start & end index
    int start_index, end_index;
    start_index = std::atoi(para_read.get_data("start_index").c_str());
    end_index = std::atoi(para_read.get_data("end_index").c_str());
    // Detector & descritor
    std::string detector, descriptor;
    detector = para_read.get_data("detector");
    descriptor = para_read.get_data("descriptor");
    // other

    int min_inliers = std::atoi(para_read.get_data("min_inliers").c_str());
    double max_norm = std::atof(para_read.get_data("max_norm").c_str());
    // Visulaze
    bool visualize_pointcloud = para_read.get_data("visualize_pointcloud") == std::string("yes");

    //
    pcl::visualization::CloudViewer viewer ("visual Odometry");
    pointcloud::Ptr cloud (new pointcloud());
    Frame last_frame;
/*
//    // Display cloud
//    CallBackArg arg;
//    pcl::visualization::PCLVisualizer::Ptr viewer = creat_visualizer(arg);
//    bool quit = false;
*/


    for(int i = start_index; i < end_index; ++i)
    {
//        viewer->removeAllPointClouds();
        Frame current_frame;

        if(i == start_index)
        {
            read_frame(para_read, start_index, last_frame);
            compute_keypoints_descriptors(last_frame, detector, descriptor);
            cloud = image_to_pointcloud(last_frame, camera);
        //    add_point_cloud<PointT>(viewer, cloud, "0", 0);
            std::cout << "last_frame " << start_index << ": " << last_frame.desp.size() << std::endl;
            continue;
        }

        std::cout << "Test i" << std::endl;
        read_frame(para_read, i, current_frame);
        compute_keypoints_descriptors(current_frame, detector, descriptor);
        std::cout << "\ncurrent_frame " << i << ": \n" << std::endl;

        // Estimate motion
        Result_of_pnp result;
        Eigen::Isometry3d trans;
        result = estimate_motion(last_frame, current_frame, camera);
        if(result.inliers < min_inliers)
        {
            std::cout << "\nInliers false.\n" << std::endl;
            continue;
        }
        if(norm_of_transform(result) >= max_norm)
        {

            continue;
        }

        trans = cvMat2eigen(result);
        std::cout << "Trans :\n" << trans.matrix() << std::endl;
        // Joint pointcloud
        cloud = joint_pointcloud(cloud, current_frame, trans, camera, para_read);

        // Show cloud
        if(visualize_pointcloud)
        {
//            add_point_cloud<PointT>(viewer, cloud, "cloud");
//            quit = viewer_wait(viewer, arg);
            viewer.showCloud(cloud, "cloud");

        }
        last_frame = current_frame;
//        cv::imshow("last_frame", last_frame.color);
//        cv::waitKey(0);
    }

    pcl::io::savePCDFile("../vo_cloud.pcd", *cloud);

    return 0;
}
