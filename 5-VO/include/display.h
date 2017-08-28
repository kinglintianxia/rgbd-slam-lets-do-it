#ifndef DISPLAY_H
#define DISPLAY_H
#include <pcl/visualization/pcl_visualizer.h>

struct CallBackArg{
  CallBackArg(pcl::visualization::PCLVisualizer::Ptr v)
      : viewer_ptr (v)
      , key (-1)
      , new_key_flag (0)
      , cloud (new pcl::PointCloud<pcl::PointXYZ>)
  {

  }
  CallBackArg()
      : key (-1)
      , new_key_flag (0)
  {

  }
  pcl::visualization::PCLVisualizer::Ptr viewer_ptr;
  int key;
  int new_key_flag;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};


extern pcl::visualization::PCLVisualizer::Ptr creat_visualizer (CallBackArg& arg);

extern std::vector<int> creat_viewport(pcl::visualization::PCLVisualizer::Ptr viewer, int row, int col);


extern bool viewer_wait(pcl::visualization::PCLVisualizer::Ptr viewer, CallBackArg& arg);

extern void remove_all(pcl::visualization::PCLVisualizer::Ptr viewer);

template<typename PointType>
void add_point_cloud(pcl::visualization::PCLVisualizer::Ptr viewer, typename pcl::PointCloud<PointType>::Ptr cloud, std::string cloud_id, int viewport = 0, double r=-1, double g=-1, double b=-1){
    if(r>=0 && g>=0 && b>=0){
        pcl::visualization::PointCloudColorHandlerCustom<PointType> color(cloud, r, g, b);
        viewer->addPointCloud(cloud, color, cloud_id, viewport);
    }
    else{
        pcl::visualization::PointCloudColorHandlerRandom<PointType> color(cloud);
        viewer->addPointCloud(cloud, color, cloud_id, viewport);
    }
    return;
}

extern void add_point_cloud_normal(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string cloud_id, int viewport = 0, double r=-1, double g=-1, double b=-1);

#endif // DISPLAY_H
