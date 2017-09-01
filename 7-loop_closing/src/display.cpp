#include "display.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* arg)
{
    struct CallbackArg* data = (struct CallbackArg *)arg;
    if (event.getPointIndex() == -1)
        return;
    pcl::PointXYZ select_point;
    event.getPoint(select_point.x, select_point.y, select_point.z);
    std::cout <<"pick point : x=" <<select_point.x << ", y=" << select_point.y << ", z=" << select_point.z << std::endl;
}

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
  std::string pressed = event.getKeySym();
  CallBackArg * arg = (CallBackArg*)data;
  if(event.keyDown ())
  {
    arg->key = pressed[0];    //for external key process in mail loop
    arg->new_key_flag = 1; //for external key process in mail loop
    if(pressed == "o")
    {
        arg->viewer_ptr->setCameraPosition(0,0,0, 0,0,1, -1,0,0 ,0);
    }
  }
}

pcl::visualization::PCLVisualizer::Ptr creat_visualizer (CallBackArg& arg)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Aqrose Main Window"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  arg.viewer_ptr = viewer;
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&arg);
  viewer->registerPointPickingCallback(pp_callback, (void*)&arg);
  viewer->spinOnce();
  return (viewer);
}

std::vector<int> creat_viewport(pcl::visualization::PCLVisualizer::Ptr viewer, int row, int col){
    double dy = 1.0/row;
    double dx = 1.0/col;
    std::vector<int> viewport_id;
    viewport_id.resize(row*col);
    for(int i=0; i<row; i++){
        for(int j=0; j<col; j++){
            viewer->createViewPort (j*dx, i*dy, (j+1)*dx, (i+1)*dy, viewport_id[i*col+j]);
            std::string text_str = "v"+std::to_string(i*col+j);
            viewer->addText (text_str, 10, 15, text_str, viewport_id[i*col+j]);
        }
    }
    return viewport_id;
}

bool viewer_wait(pcl::visualization::PCLVisualizer::Ptr viewer, CallBackArg& arg){
    while(!viewer->wasStopped ()){ // press 'n' to continue
        if(arg.new_key_flag){        // new key to process
            arg.new_key_flag = 0;
            if(arg.key=='n' || arg.key=='s'){       // 'n' to continue
                if(arg.key=='s'){
                    static int cnt=0;
                    pcl::PCDWriter writer;
                    std::string save_name = "cloud_saved_"+std::to_string(cnt)+".pcd";
                    writer.writeBinary(save_name, *arg.cloud);
                    std::cout << "saved : " << save_name << std::endl;
                    cnt++;
                }
                return false;
                break;
            }
        }
        viewer->spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    return true;
}

void remove_all(pcl::visualization::PCLVisualizer::Ptr viewer){
//    viewer->removeAllCoordinateSystems();
    viewer->removeAllShapes();
    viewer->removeAllPointClouds();
}

void add_point_cloud_normal(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string cloud_id, int viewport, double r, double g, double b){
    add_point_cloud<pcl::PointNormal>(viewer, cloud, cloud_id, viewport, r, g, b);
    viewer->addPointCloudNormals<pcl::PointNormal>(cloud, 1, 0.02, cloud_id+"normals", viewport);
    return;
}
