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
// G2O
#include <g2o/types/slam3d/types_slam3d.h> //顶点类型
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


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
//    std::cout << "Read current frame: " << ss.str() << std::endl;
}

double norm_of_transform(Result_of_pnp result)
{
    return fabs(std::min(cv::norm(result.rvect), (2 * M_PI - cv::norm(result.rvect))) + fabs(cv::norm(result.tvect)));

}


int main(int argc, char** argv)
{
    // Get parameter
    ParameterReader para_read("../parameters.txt");
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

    ////////////----G2O-----///////////////////
    // Optimize type
    typedef g2o::BlockSolver_6_3 slam_block_solver;
    typedef g2o::LinearSolverCSparse<slam_block_solver::PoseMatrixType> slam_linear_solver;
    // Solver init
    slam_linear_solver* linear_solver = new slam_linear_solver();
    linear_solver->setBlockOrdering(false);
    slam_block_solver* block_solver = new slam_block_solver(linear_solver);
    g2o::OptimizationAlgorithmLevenberg* solver (new g2o::OptimizationAlgorithmLevenberg(block_solver));

    g2o::SparseOptimizer global_optimizer;
    global_optimizer.setAlgorithm(solver);
    // No debug print
    global_optimizer.setVerbose(false);
    ///////////////////////////////////////////

    int last_index = start_index;


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

            // add first g2o vertex
            g2o::VertexSE3* v = new g2o::VertexSE3();
            v->setId(i);
            v->setEstimate(Eigen::Isometry3d::Identity());
            v->setFixed(true);  // first vertex fixed
            global_optimizer.addVertex(v);

            continue;
        }

//        std::cout << "Test i" << std::endl;
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
//        cloud = joint_pointcloud(cloud, current_frame, trans, camera, para_read);

        // add next g2o vertex
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId(i);
        v->setEstimate(Eigen::Isometry3d::Identity());
        global_optimizer.addVertex(v);

        // Edge
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // Link two vertex
        edge->vertices()[0] = global_optimizer.vertex(last_index);
        edge->vertices()[1] = global_optimizer.vertex(i);
        // Imformation matrix
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为1/0.01 = 100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        edge->setInformation(information);
        // 边的估计即是pnp求解之结果
        edge->setMeasurement(trans);
        global_optimizer.addEdge(edge);


        // Show cloud
//        if(visualize_pointcloud)
//        {

//            viewer.showCloud(cloud, "cloud");

//        }

        last_frame = current_frame;
        last_index = i;


    }

//    pcl::io::savePCDFile("../vo_cloud.pcd", *cloud);
    // 优化所有边
    std::cout<<"optimizing pose graph, vertices: "<<global_optimizer.vertices().size()<<std::endl;
    global_optimizer.save("../result_before.g2o");
    global_optimizer.initializeOptimization();
    global_optimizer.optimize( 100 ); //可以指定优化步数
    global_optimizer.save( "../result_after.g2o" );
    std::cout<<"Optimization done."<<std::endl;

    global_optimizer.clear();


    return 0;
}
