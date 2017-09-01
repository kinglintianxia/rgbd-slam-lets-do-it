#include "slam_base.h"
#include "display.h"
// stdlib
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>



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
//    pcl::visualization::CloudViewer viewer ("visual Odometry");
    pointcloud::Ptr cloud (new pointcloud());
    Frame last_frame;
/*
//    // Display cloud
//    CallBackArg arg;
//    pcl::visualization::PCLVisualizer::Ptr viewer = creat_visualizer(arg);
//    bool quit = false;
*/

    ////////////----G2O-----///////////////////

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


    // Key frames
    std::vector<Frame> keyframes;
    double keyframe_threshold = atof( para_read.get_data("keyframe_threshold").c_str() );
    bool check_loop_closure = para_read.get_data("check_loop_closure")==std::string("yes");


    for(int i = start_index; i < end_index; ++i)
    {
//        viewer->removeAllPointClouds();
        Frame current_frame;

        if(i == start_index)
        {
            read_frame(para_read, i, last_frame);
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
            // First keyframe
            keyframes.push_back(last_frame);
            std::cout << "Initialize Done." << std::endl;
            continue;
        }

//        std::cout << "Test i" << std::endl;
        read_frame(para_read, i, current_frame);
        compute_keypoints_descriptors(current_frame, detector, descriptor);
        std::cout << "\ncurrent_frame " << i << ": \n" << std::endl;
        /**
         *  back()
         *  Returns a read-only (constant) reference to the data at the
         *  last element of the %vector.
         */
        CHECK_RESULT keyframe_res = check_key_frames(keyframes.back(), current_frame, global_optimizer, para_read);

        switch (keyframe_res) // 根据匹配结果不同采取不同策略
        {
        case NOT_MATCHED:
            //没匹配上，直接跳过
            std::cout<<RED"Not enough inliers."<<std::endl;
            break;
        case TOO_FAR:
            // 太近了，也直接跳
            std::cout<<RED"Too far away, may be an error."<<std::endl;
            break;
        case TOO_CLOSE:
            // 太远了，可能出错了
            std::cout<<RESET"Too close, not a keyframe"<<std::endl;
            break;
        case KEYFRAME:
            std::cout<<GREEN"This is a new keyframe"<<std::endl;
            // 不远不近，刚好
            /**
             * This is important!!
             * This is important!!
             * This is important!!
             * (very important so I've said three times!)
             */
            // 检测回环
            if (check_loop_closure)
            {
                check_nearby_loops( keyframes, current_frame, global_optimizer, para_read);
                check_random_loops( keyframes, current_frame, global_optimizer, para_read);
            }
            keyframes.push_back( current_frame );
            break;

        default:
            break;
        }

//        // Estimate motion
//        Result_of_pnp result;
//        Eigen::Isometry3d trans;
//        result = estimate_motion(last_frame, current_frame, camera);
//        if(result.inliers < min_inliers)
//        {
//            std::cout << "\nInliers false.\n" << std::endl;
//            continue;
//        }
//        if(norm_of_transform(result) >= max_norm)
//        {

//            continue;
//        }

//        trans = cvMat2eigen(result);
//        std::cout << "Trans :\n" << trans.matrix() << std::endl;

//        // Joint pointcloud
////        cloud = joint_pointcloud(cloud, current_frame, trans, camera, para_read);

        // add next g2o vertex
//        g2o::VertexSE3* v = new g2o::VertexSE3();
//        v->setId(i);
//        v->setEstimate(Eigen::Isometry3d::Identity());
//        global_optimizer.addVertex(v);

//        // Edge
//        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
//        // Link two vertex
//        edge->vertices()[0] = global_optimizer.vertex(last_index);
//        edge->vertices()[1] = global_optimizer.vertex(i);
//        // Imformation matrix
//        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
//        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
//        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
//        // 那么协方差则为对角为0.01的矩阵，信息阵则为1/0.01 = 100的矩阵
//        information(0,0) = information(1,1) = information(2,2) = 100;
//        information(3,3) = information(4,4) = information(5,5) = 100;
//        edge->setInformation(information);
//        // 边的估计即是pnp求解之结果
//        edge->setMeasurement(trans);
//        global_optimizer.addEdge(edge);


//        // Show cloud
////        if(visualize_pointcloud)
////        {

////            viewer.showCloud(cloud, "cloud");

////        }

//        last_frame = current_frame;
//        last_index = i;


    }
    std::cout << "Key Frame size: " << keyframes.size() << std::endl;

//    pcl::io::savePCDFile("../vo_cloud.pcd", *cloud);
    // 优化所有边
    std::cout<<"optimizing pose graph, vertices: "<<global_optimizer.vertices().size()<<std::endl;
    global_optimizer.save("../result_before.g2o");
    global_optimizer.initializeOptimization();
    global_optimizer.optimize( 100 ); //可以指定优化步数
    global_optimizer.save( "../result_after.g2o" );
    std::cout<<"Optimization done."<<std::endl;


    // 拼接点云地图
    std::cout<<"saving the point cloud map..."<<std::endl;

    pointcloud::Ptr output ( new pointcloud() ); //全局地图
    pointcloud::Ptr tmp ( new pointcloud() );

    pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
    pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 ); //4m以上就不要了

    double gridsize = std::atof( para_read.get_data("voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    // Use optimized trans to connect cloud
    for (size_t i=0; i<keyframes.size(); i++)
    {
        // 从g2o里取出一帧
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(global_optimizer.vertex( keyframes[i].id ));
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
        pointcloud::Ptr newCloud = image_to_pointcloud(keyframes[i], camera ); //转成点云
        // 以下是滤波
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // 把点云变换后加入全局地图中
        // New cloud add to origin cloud, use T.inverse(), diff from lession 6 !!!
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );
    //存储
    pcl::io::savePCDFile( "../slam_cloud.pcd", *tmp );

    cout<<"Final map is saved."<<endl;

    global_optimizer.clear();


    return 0;
}
