#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

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
// 更好的写法是存到参数文件中，但为方便起见我就直接这样做了
float camera_scale  = 1000;
float camera_cx     = 325.5;
float camera_cy     = 253.5;
float camera_fx     = 518.0;
float camera_fy     = 519.0;

int main(int argc, char** argv)
{
    // Read keyframes
    std::ifstream fin("../data/keyframe.txt");
    std::vector<int> keyframes;
    if(!fin)
        std::cout << "Can not open keyframe.txt" << std::endl;
    while(!fin.eof())
    {
        int num;
        fin >> num;     // get oneline
        if(fin.fail())
            break;
        keyframes.push_back(num);
    }
    fin.close();
    std::cout << "Keyframe size: " << keyframes.size() << std::endl;

    // Read trajectory
    fin.open("../data/trajectory.txt");
    std::vector<Eigen::Affine3d> pose;

    if(!fin)
        std::cout << "Can not open trajectory.txt" << std::endl;
    while(!fin.eof())
    {
        int keyframe;
        float quta[7];
        fin >> keyframe;
        for(int i = 0; i < 7; i ++)
            fin >> quta[i];
        if(fin.fail())
            break;
        // Trans
        Eigen::Quaterniond q(quta[6], quta[3], quta[4], quta[5]);
        Eigen::Affine3d trans(q);
        trans.translation() << quta[0], quta[1], quta[2];
        pose.push_back(trans);
    }
    fin.close();
    std::cout << "pose size: " << pose.size() << std::endl;

    // Color octomap
    octomap::ColorOcTree tree(0.05);
    pcl::visualization::CloudViewer viewer("cloud");

    for(int i = 0; i < keyframes.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        boost::format fmt("../data/rgb_index/%d.ppm");
        cv::Mat rgb = cv::imread((fmt % keyframes[i]).str().c_str());

        fmt = boost::format("../data/dep_index/%d.pgm");
        cv::Mat depth = cv::imread((fmt % keyframes[i]).str().c_str(), -1);
        std::cout << "rgb size: " << rgb.size() << "\ndepth size: " << depth.size() << std::endl;

        for(int v = 0; v < depth.rows; v++)
        {
            for(int u = 0; u < depth.cols; u++)
            {
                ushort d = depth.ptr<ushort>(v)[u];
                if(d ==0)
                    continue;
                pcl::PointXYZRGB point;
                point.z = (double)d / camera_scale;
                point.x = (u - camera_cx) * point.z / camera_fx;
                point.y = (v - camera_cy) * point.z / camera_fy;

                point.b = rgb.ptr<uchar>(v)[u*3];
                point.g = rgb.ptr<uchar>(v)[u*3 + 1];
                point.r = rgb.ptr<uchar>(v)[u*3 + 2];

                cloud->points.push_back(point);
            }

        }

        std::cout << "cloud size: " << cloud->points.size() << std::endl;

        viewer.showCloud(cloud);
        sleep(2.0);
        // octomap
        pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
        pcl::transformPointCloud(*cloud, temp_cloud, pose[i].matrix());
        // octo cloud
        octomap::Pointcloud octo_cloud;
        for(auto p:temp_cloud.points)
            octo_cloud.push_back(p.x, p.y, p.z);

        tree.insertPointCloud(octo_cloud, octomap::point3d(pose[i](0, 3), pose[i](1, 3), pose[i](2, 3)));

        for(auto p:temp_cloud.points)
            tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);


    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.write("../data/joint_octomap.ot");
    std::cout << "Done." << std::endl;

    return 0;
}

