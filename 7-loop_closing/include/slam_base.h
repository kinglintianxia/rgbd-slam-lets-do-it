# pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// G2O
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */



typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> pointcloud;

// Optimize type
typedef g2o::BlockSolver_6_3 slam_block_solver;
typedef g2o::LinearSolverEigen<slam_block_solver::PoseMatrixType> slam_linear_solver;

enum CHECK_RESULT {NOT_MATCHED = 0, TOO_FAR, TOO_CLOSE, KEYFRAME};

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
    int id;
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
    int inliers;

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
            std::cout <<"Parameter filename does not exist!" << std::endl;
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
// cvMat to Eigen
Eigen::Isometry3d cvMat2eigen(Result_of_pnp result);

// Get camera parameter
void get_cam_param(ParameterReader reader, CAMERA_INTRINSIC_PARAMETERS& camera);

// Joint point cloud
pointcloud::Ptr joint_pointcloud(pointcloud::Ptr origin, Frame newframe, Eigen::Isometry3d trans, CAMERA_INTRINSIC_PARAMETERS camera, ParameterReader read);

//
void read_frame(ParameterReader read, int cnt, Frame& frame);

//
double norm_of_transform(Result_of_pnp result);

//
CHECK_RESULT check_key_frames(Frame& frame1, Frame& frame2, g2o::SparseOptimizer& optimize, ParameterReader pd, bool is_loops = false);

//
void check_nearby_loops(std::vector<Frame>& frames, Frame& currentFrame, g2o::SparseOptimizer& optimize, ParameterReader pd);

//
void check_random_loops(std::vector<Frame>& frames, Frame& currentFrame, g2o::SparseOptimizer& optimize, ParameterReader pd);
