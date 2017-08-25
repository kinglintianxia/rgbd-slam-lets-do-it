#include "slam_base.h"

pointcloud::Ptr image_to_pointcloud(cv::Mat color, cv::Mat depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
    pointcloud::Ptr cloud;
    for(int v = 0; v < depth.rows; ++v)
    {
        for(int u = 0; u < depth.cols; ++u)
        {
            ushort d = depth.ptr<ushort>(v)[u];
            if(d == 0)
                continue;

            PointT point;
            point.z = (double)d / camera.scale;
            point.x = (u - camera.cx) * point.z / camera.fx;
            point.y = (v - camera.cy) * point.z / camera.fy;

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
