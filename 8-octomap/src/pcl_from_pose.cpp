#include <iostream>
#include <vector>
#include <string>
#include<fstream>
#include<algorithm>
#include <sstream>
#include<chrono>
#include <time.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

// octomap
#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap/ColorOcTree.h>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// Boost
#include <boost/format.hpp>

// 全局变量：相机矩阵
float camera_scale  = 5000.0; // Depth map values factor
float camera_cx     = 325.5;
float camera_cy     = 253.5;
float camera_fx     = 518.0;
float camera_fy     = 519.0;

// load color & depth pairs
void LoadImagePairs(const std::string &ass_file,
                    const std::string& color_img_name,
                    std::vector<std::pair<std::string,std::string>> &img_name_pairs)
{
    std::ifstream fAssociation;
    fAssociation.open(ass_file);
    // pairs
    std::pair<std::string,std::string> img_pair;
    while(!fAssociation.eof())
    {
        std::string s;
        std::getline(fAssociation,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string color_img;
            std::string tmp_str;
//            std::string sRGB, sD;
            ss >> color_img;    // color: 1341847980.854676
            if (color_img == color_img_name)
            {
                img_pair.first = color_img;
                ss >> tmp_str;   // color: rgb/1341847980.854676.png
                ss >> img_pair.second;    // depth timestamp
                ss >> tmp_str;  // depth: depth/1341847980.854690.png
                img_name_pairs.push_back(img_pair);
            }
            else
                continue;
        }
    }
}

int main(int argc, char** argv)
{
    // Read keyframes
    std::ifstream fin("../data/KeyFrameTrajectory.txt");
    // associations.txt
//    std::ifstream ass_file;
//    ass_file.open("/home/king/Documents/image/rgbd_dataset_freiburg3_long_office_household/associations.txt");
    std::string ass_file = "/home/king/Documents/image/rgbd_dataset_freiburg3_long_office_household/associations.txt";
    std::vector<std::string> keyframes;
    std::vector<Eigen::Affine3d> pose;
    // string pairs
    std::vector<std::pair<std::string,std::string>> img_name_pairs;
    if(!fin)
        std::cout << "Can not open trajectory.txt" << std::endl;
    while(!fin.eof())
    {
        std::string kf_name;
        float quta[7];
        fin >> kf_name;
        LoadImagePairs(ass_file, kf_name, img_name_pairs);
        keyframes.push_back(kf_name);
        // quta
        for(int i = 0; i < 7; i ++)
            fin >> quta[i];
        if(fin.fail())
            break;
        // Trans
        Eigen::Quaterniond q(quta[6], quta[3], quta[4], quta[5]);   // Quaterniond(w,x,y,z)
        Eigen::Affine3d trans(q);
        trans.translation() << quta[0], quta[1], quta[2];
        pose.push_back(trans);
    }
    fin.close();
//    ass_file.close();
    std::cout << "pose size: " << pose.size() << std::endl;

    // pcl cloud
    pcl::visualization::CloudViewer viewer("cloud");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr joint_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i = 0; i < img_name_pairs.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        boost::format fmt("/home/king/Documents/image/tum3_seg/rgb/%s.png");
        cv::Mat rgb = cv::imread((fmt % keyframes[i]).str().c_str());

        fmt = boost::format("/home/king/Documents/image/rgbd_dataset_freiburg3_long_office_household/depth/%s.png");
        cv::Mat depth = cv::imread((fmt % img_name_pairs[i].second).str().c_str(), -1);
        std::cout << "rgb size: " << rgb.size() << "\ndepth size: " << depth.size() << std::endl;
        if (rgb.empty() | depth.empty())
        {
            std::cout << "---Empty image!---" << std::endl;
            continue;
//            exit(1);
        }
        for(int v = 0; v < depth.rows; v++)
        {
            for(int u = 0; u < depth.cols; u++)
            {
                ushort d = depth.ptr<ushort>(v)[u];
                if(d ==0)
                    continue;
                pcl::PointXYZRGB point;
                point.z = (double)d / camera_scale;
                if (point.z < 0.01 || point.z > 10)
                    continue;
                point.x = (u - camera_cx) * point.z / camera_fx;
                point.y = (v - camera_cy) * point.z / camera_fy;

                point.b = rgb.ptr<uchar>(v)[u*3];
                point.g = rgb.ptr<uchar>(v)[u*3 + 1];
                point.r = rgb.ptr<uchar>(v)[u*3 + 2];

                cloud->points.push_back(point);
            }
        }

        std::cout << "cloud size: " << cloud->points.size() << std::endl;

        cloud->is_dense = false;
        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.05f, 0.05f, 0.05f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
        sor.filter (*tmp);  // DO NOT FILTER IN_PLACE!!!
        // transform cloud
        pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
        pcl::transformPointCloud(*tmp, temp_cloud, pose[i].matrix());
        // joint cloud
        *joint_cloud += temp_cloud;
        viewer.showCloud(joint_cloud);
//        sleep(0.5);
    }

    // Write pointcloud
    pcl::PCDWriter writer;
    writer.write("../data/joint_pose_cloud.pcd", *joint_cloud);
    std::cout << "Done." << std::endl;
    // for view
//    int key = cv::waitKey(30)&255;
//    if(key != 27)	// esc
//    {
//        viewer. spinOnce();
//    }

    return 0;
}

