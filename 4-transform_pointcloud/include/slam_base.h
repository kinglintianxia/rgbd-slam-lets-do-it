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
    double cx, cy;
    double fx, fy;
    double scale;  // camera scale facotr default:1000

};


// Image frame
struct Frame
{
    cv::Mat color;
    cv::Mat depth;
    cv::Mat desp;
    std::vector<cv::KeyPoint> kp;

};

// Result of pnp
struct Result_of_pnp
{
    cv::Mat rvect;
    cv::Mat tvect;
    cv::Mat inliers;

};



// Parameter reader
class ParameterReader
{
public:
    // Constructor
    ParameterReader(std::string filename)
    {
        std::ifstream fin(filename.c_str());
        if(!fin)
            std::cout <<"filename does not exist!" << std::endl;
        while(!fin.eof())
        {
            std::string str;
            getline(fin, str);
            // jump comment line
            if(str[0] == '#')
                continue;
            int position = str.find("=");
            if(position == -1)
                continue;
            else
            {
                std::string key = str.substr(0, position);
                std::string value = str.substr(position+1, str.length());
                data[key] = value;
            }
            if(!fin.good())
                break;

        }


    }

    // Get data method
    std::string get_data(std::string key)
    {
        std::map<std::string, std::string>::iterator iter = data.find(key);
        if(iter == data.end())
        {
            std::cout << "Can not find key: " << key << std::endl;
            return std::string("Not found.");
        }
        else
            return iter->second;

    }

private:
    std::string file_name_;
    std::map<std::string, std::string> data;

};

pointcloud::Ptr image_to_pointcloud(Frame frame, CAMERA_INTRINSIC_PARAMETERS camera);

cv::Point3f point2d_to_3d(cv::Point3f point, CAMERA_INTRINSIC_PARAMETERS camera);


// Compute keypoints and descriptors
void compute_keypoints_descriptors(Frame& frame, std::string det, std::string des);
// Get result of motion
Result_of_pnp estimate_motion(Frame frame1, Frame frame2, CAMERA_INTRINSIC_PARAMETERS camera);


