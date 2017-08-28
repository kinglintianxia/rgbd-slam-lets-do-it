# VO 坑列表
---
1. 深度图读取要加 **-1**
> frame.depth = cv::imread(ss.str(), -1);

2. last_frame 放循环外， current_frame 放循环里，每次清零。

3. result.inliers = inliers.rows;

4. trans.translation() << result.tvect.at<double>(*0*, 0), (*1*, 0), (*2*, 0);

5. VoxelGrid filter 要赋值给新的cloud
>  pointcloud::Ptr temp_cloud (new pointcloud());
   sor.filter(*temp_cloud);

6. 加入最小distance限制
> if ( min_dis < 10 )
     min_dis = 10;

7. std::stringstream ss; ss清零不能用ss.clear(),无效。要用赋值空清零。
>  ss.str("");

8.  pcl::transformPointCloud(*origin, *trans_cloud, **trans.matrix()**);
