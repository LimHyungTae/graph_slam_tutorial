# Conversion

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)   


----------------

## Conversion이 중요한 이유?

ROS에서 [geometry_msgs/Pose](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html)로 데이터를 주기 때문에,

xyz-roll-pitch-yaw(xyzrpy)
xyz-quaternion(geoPose)
4x4 transformation matrix(eigen)

```cpp
cv::Mat xyzrpy2mat(float x, float y, float z, float roll, float pitch, float yaw)
    {
          cv::Mat rot_vec = cv::Mat::zeros(3,1,CV_32FC1);
          rot_vec.at<float>(0) = roll;
          rot_vec.at<float>(1) = pitch;
          rot_vec.at<float>(2) = yaw;

          cv::Mat rot_mat;
          cv::Rodrigues(rot_vec,rot_mat);

          cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);

          rot_mat.copyTo(result(cv::Rect(0,0,3,3)));

          result.at<float>(0,3) = x;
          result.at<float>(1,3) = y;
          result.at<float>(2,3) = z;

          result.at<float>(3,3) = 1;

          return result;
    }
```
xyz-roll-pitch-yaw(xyzrpy) 
