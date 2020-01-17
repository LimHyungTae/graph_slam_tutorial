# PCL cheat sheet(2/2)

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)

I really want to thank Seungwon and Hyungjin for their codes. <br/> 
Seungwon Song (sswan55@kaist.ac.kr) <br/>
Hyungjin Kim (hjkim86@kaist.ac.kr)

-----------

## 형 변환(Type Conversion)

ROS상에서 LiDAR 센서들은 sensor_msgs::PointCloud2나 Sensor_msgs::LaserScan(2D LiDAR의 경우) 타입을 통해 데이터가 들어오기 때문에, pcl을 통해 pointcloud를 다루기 위해서는 pcl::PointCloud로 형변환을 해줘야 한다.

### sensor_msgs::PointCloud2 :arrow_right: pcl::PointCloud
```cpp
pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
  }
```
### pcl::PointCloud :arrow_right: sensor_msgs::PointCloud2
```cpp
sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZ> cloud_src)
  {
    sensor_msgs::PointCloud2 cloudmsg;
    pcl::toROSMsg(cloudsrc, cloudmsg);
    cloudmsg.header.frame_id = "map";
    return cloudmsg;
  }
```

### sensor_msgs::LaserScan :arrow_right: sensor_msgs::PointCloud2
```cpp

#include "laser_geometry/laser_geometry.h"

sensor_msgs::PointCloud2 laser2cloudmsg(sensor_msgs::LaserScan laser)
    {
      static laser_geometry::LaserProjection projector;
      sensor_msgs::PointCloud2 pc2_dst;
      projector.projectLaser(laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
      pc2_dst.header.frame_id = "map";

      return pc2_dst;
    }
```

## 변환 행렬 곱하기(Transformation)
![tf](../img/pcl_robot_sensor.PNG)

로보틱스에는 다양한 좌표계가 존재한다. 위의 사진의 경우만 봐도 센서 데이터는 센서의 원점을 기준으로 데이터를 얻는다. 하지만 우리가 실제로 요하는 것 로봇의 body 기준의 정보들이기 때문에, 센서 데이터를 취득한 후 원하는 좌표계에 변환(transformation)을 해주어야 한다. 변환을 할  아래와 같은 코드를 사용하면 Eigen::Matrix4f trans만큼 모든 포인트클라우드가 변환된다.
```cpp
//Input: pcl::PointCloud source, cloud_src
//Output: Transformed pcl::PointCloud, pc_transformed via 4x4 transformation matrix
#include <pcl/common/transforms.h>

pcl::PointCloud<pcl::PointXYZ> pc_transformed;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZ>);

Eigen::Matrix4f trans;
trans<< 1,   0,  0, 0.165,
        0,   1,  0, 0.000,
        0,   0,  1, 0.320,
        0,   0,  0,     1;
pcl::transformPointCloud(cloud_src, *ptr_transformed, trans);

pc_transformed = *ptr_transformed
```

## 지정 축에 대해 포인트 클라우드 필터링하기(Filtering using a PassThrough Filter)

```cpp
#include <pcl/filters/passthrough.h>

//Input: pcl::PointCloud source, cloud_src
//Output: Filtered pcl::PointCloud, pc_filtered along z axis, from 0.5m to 100.0m

pcl::PointCloud<pcl::PointXYZ> pc_filtered;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PassThrough<pcl::PointXYZ> filter;

double min_range = 0.5;
double max_range = 100.0;
*ptr_filtered = cloud_src;

filter.setInputCloud(ptr_filtered);
filter.setFilterFieldName("z");
filter.setFilterLimits(min_range, max_range);
// filter.setFilterLimitsNegative(true);
filter.filter(*ptr_filtered);

pc_filtered = *ptr_filtered;
```

## Downsampling to a Voxel Grid

![centroid](../img/pcl_centroid.PNG)
```cpp
#include <pcl/filters/voxel_grid.h>

//Input: pcl::PointCloud source, cloud_src
//Output: voxelized pcl::PointCloud, pc_voxelized 

pcl::PointCloud<pcl::PointXYZ> pc_voxelized;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

double var_voxelsize = 0.05;

*ptr_filtered = cloud_src;
voxel_filter.setInputCloud(ptr_filtered);
voxel_filter.setLeafSize(var_voxelsize, var_voxelsize, var_voxelsize);
voxel_filter.filter(*ptr_filtered);

pc_voxelized = *ptr_filtered;
```
그런데 굳이 filter()함수에 ptr을 넣지 않고 직접적으로 pcl::PointCloud<pcl::PointXYZ>로 받아도 된다.
```cpp
void mapgen::voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size){

  static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pc_src);
  voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
  voxel_filter.filter(pc_dst);

}
```

## Statistical Outlier Removal
![sor](../img/pcl_sor.PNG)

The number of neighbors to analyze for each point is set to 10, and the standard deviation multiplier to 1.0
```cpp
#include <pcl/filters/statistical_outlier_removal.h>

//Input: pcl::PointCloud source, cloud_src
//Output: voxelized pcl::PointCloud, pc_sor_filtered 

pcl::PointCloud<pcl::PointXYZ> pc_sor_filtered;
pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
*ptr_sor_filtered = cloud_src;

int num_neigbor_points = 10;
double std_multiplier = 1.0;

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (ptr_sor_filtered);
sor.setMeanK (num_neigbor_points);
sor.setStddevMulThresh (std_multiplier);
sor.filter(*ptr_sor_filtered);

pc_sor_filtered = *ptr_sor_filtered;
```

## Reference
[1] Using a matrix to transform a point cloud, http://pointclouds.org/documentation/tutorials/matrix_transform.php <br/>
[2] Filtering a PointCloud using a PassThrough filter, http://pointclouds.org/documentation/tutorials/passthrough.php <br/>
[3] Downsampling a PointCloud using a VoxelGrid filter, http://pointclouds.org/documentation/tutorials/voxel_grid.php <br/>
[4] Removing outliers using a StatisticalOutlierRemoval filter, http://pointclouds.org/documentation/tutorials/statistical_outlier.php <br/>



