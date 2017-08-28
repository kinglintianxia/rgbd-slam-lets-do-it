#include "slam_base.h"

pointcloud::Ptr image_to_pointcloud(Frame frame, CAMERA_INTRINSIC_PARAMETERS camera)
{
    // !!!!!!!!!!!!!Segmentation fault (core dumped)!!!!!!!!!!!!!!!!!!!!!!
    pointcloud::Ptr cloud (new pointcloud());
    for(int v = 0; v < frame.depth.rows; ++v)
    {
        for(int u = 0; u < frame.depth.cols; ++u)
        {
            ushort d = frame.depth.ptr<ushort>(v)[u];
            if(d == 0)
                continue;

            PointT point;
            point.z = double(d) / camera.scale;
            point.x = (u - camera.cx) * point.z / camera.fx;
            point.y = (v - camera.cy) * point.z / camera.fy;

            // Color
            point.b = frame.color.ptr<uchar>(v)[u * 3];
            point.g = frame.color.ptr<uchar>(v)[u * 3 + 1];
            point.r = frame.color.ptr<uchar>(v)[u * 3 + 2];

            cloud->points.push_back(point);

        }

    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
//    cloud->resize(cloud->width * cloud->height);
    cloud->is_dense = false;

//    std::cout << "cloud: " << cloud->points[10].x << std::endl;

    return cloud;

}

cv::Point3f point2d_to_3d(cv::Point3f point, CAMERA_INTRINSIC_PARAMETERS camera)
{
    cv::Point3f p;

    p.z = (double)point.z / camera.scale;
    p.x = (point.x - camera.cx)* p.z /camera.fx;
    p.y = (point.y - camera.cy)* p.z /camera.fy;
    return p;

}

void compute_keypoints_descriptors(Frame& frame, std::string det, std::string des)
{
    // OpenCV2
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(det);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create(des);
    if (!detector || !descriptor)
    {
        std::cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<std::endl;
        return;
    }
    // keypoints
    detector->detect(frame.color, frame.kp);
    // descriptors
    descriptor->compute(frame.color, frame.kp, frame.desp);

}

Result_of_pnp estimate_motion(Frame frame1, Frame frame2, CAMERA_INTRINSIC_PARAMETERS camera)
{
    // Compute matches
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    Result_of_pnp result;

    matcher.match(frame1.desp, frame2.desp, matches);
    std::cout << "Total matches: " <<matches.size() << std::endl;
    // Compute good matches
    std::vector<cv::DMatch> good_matches;
    double min_dis = 9999;
    for(size_t i = 0; i< matches.size(); ++i)
    {
        if(matches[i].distance < min_dis)
                min_dis = matches[i].distance;
    }
    if ( min_dis < 10 )
            min_dis = 10;
    std::cout << "min_dis: " <<min_dis<< std::endl;
    // Filter good matches
    for (size_t i = 0; i < matches.size(); ++i)
    {
        if(matches[i].distance < 10* min_dis)
            good_matches.push_back(matches[i]);

    }
    std::cout << "good_matches: " <<good_matches.size() << std::endl;
    if(good_matches.size() <= 10)
    {
        result.inliers = -1;
        return result;

    }
    // construct Points 3d and 2d
    std::vector<cv::Point3f> pt3;
    std::vector<cv::Point2f> pt2;
    for (size_t i=0; i<good_matches.size(); i++)
    {
        // query 是第一个image, train 是第二个image
        cv::Point2f p = frame1.kp[good_matches[i].queryIdx].pt;
        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2d_to_3d(pt, camera);
        pt3.push_back( pd );
        // Image2 pt2
        pt2.push_back( cv::Point2f( frame2.kp[good_matches[i].trainIdx].pt ) );

    }

    // Estimate motion
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );

    // 求解pnp
    cv::Mat inliers;
    cv::solvePnPRansac( pt3, pt2, cameraMatrix, cv::Mat(), result.rvect, result.tvect, false, 100, 1.0, 100, inliers );
    result.inliers = inliers.rows;
    std::cout << "Inliers: " << result.inliers << std::endl;
//    std::cout << "result.rvect: " <<result.rvect << std::endl;
    return result;

}

// CV Mat to Eigen
Eigen::Isometry3d cvMat2eigen(Result_of_pnp result)
{
    cv::Mat r_mat;
    Eigen::Matrix3d r_eigen;
    Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
    cv::Rodrigues(result.rvect, r_mat);
    cv::cv2eigen(r_mat, r_eigen);

    trans.rotate(r_eigen);
    // at<double>(0, 0); (1, 0) ; (2, 0)
    trans.translation() << result.tvect.at<double>(0, 0), result.tvect.at<double>(1, 0), result.tvect.at<double>(2, 0);

    return trans;


}

// Get camera parameter
void get_cam_param(ParameterReader reader, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    camera.fx = std::atof(reader.get_data("camera.fx").c_str());
    camera.fy = std::atof(reader.get_data("camera.fy").c_str());
    camera.cx = std::atof(reader.get_data("camera.cx").c_str());
    camera.cy = std::atof(reader.get_data("camera.cy").c_str());
    camera.scale = std::atof(reader.get_data("camera.scale").c_str());

}

pointcloud::Ptr joint_pointcloud(pointcloud::Ptr origin, Frame newframe, Eigen::Isometry3d trans, CAMERA_INTRINSIC_PARAMETERS camera, ParameterReader read)
{
    pointcloud::Ptr trans_cloud (new pointcloud()), cloud_new (new pointcloud());

    cloud_new = image_to_pointcloud(newframe, camera);
    pcl::transformPointCloud(*origin, *trans_cloud, trans.matrix());
    *cloud_new += *trans_cloud;

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_new);
    double leafsize = std::atof(read.get_data("voxel_grid").c_str());
    sor.setLeafSize(leafsize, leafsize, leafsize);
    pointcloud::Ptr temp_cloud (new pointcloud());
    sor.filter(*temp_cloud);

    return temp_cloud;

}
