# Setting of Ouster OS1-64

## 현 Ouster사에서 제공하는 driver를 통해 데이터를 받아보자 

작성자: 임형태(shapelim@kaist.ac.kr), 박주현(mcvjoohyun@kaist.ac.kr)

----

예시 Ouster Rosbag 파일은 [여기](https://www.dropbox.com/s/9gofcgfzaa8oyft/ouster_example.bag?dl=0)를 통해 다운 가능하다(2020-01-20 기준)

## How to install(세팅 하는 법)

1. 먼저, Ouster 사에서 제공하는 driver를 catkin_ws/src 내에서 git clone한다. (2020-01-09 기준)

<pre><code>$ cd /home/$usr_name/catkin_ws/src</code>
<code>$ git clone https://github.com/ouster-lidar/ouster_example</code></pre>

2. 그 후 catkin_ws로 이동하여 complie해준다.

<pre><code>$ cd /home/$usr_name/catkin_ws</code>
<code>$ catkin_make ouster_ros</code></pre>

혹은, catkin-tools를 이용하면 아래와 같이 컴파일 하면 된다.
<pre><code>$ catkin build ouster_ros</code></pre>

## 드라이버를 통한 packet 파싱하기

1. [여기](https://www.dropbox.com/s/9gofcgfzaa8oyft/ouster_example.bag?dl=0)를 통해 다운받은 bag 파일을 rosbag play를 통해서 실행을 시킨다.

<pre><code>rosbag play ouster_example.bag --clock</code></pre>

2. 그 후, driver상에서 제공해주는 launch file을 통해 data packet을 Ros 메세지로 변환할 수 있다.

<pre><code>roslaunch /home/($user_name)/catkin_ws/src/ouster_example-master/ouster_ros/os1.launch replay:=true</code></pre>
    
**그런데...막상 launch file을 실행하면 아래와 같이 md5sum 에러가 발생한다!!!!**
    
![error_occurs](/readme_materials/ouster_error.png)    
    
## 문제가 발생하는 이유?

Ouster 사에서 제공하는 driver 내의 패킷 메세지 내에 msg 상에 header가 포함되어 있지 않기 때문에 발생하는 것으로 보인다. 그로 인해서, packet data를 sensor_msgs로 변환할 때 ROS 상에서는 data 시간에 대한 정보가 없는 패킷을 받기 때문에 error가 발생시키는 것이다. 

다시 말하자면, **Ouster사에서 제공하는 드라이버/(혹은 데이터를 패킷으로 생성할 때) 내에서 TimeStamp를 찍어주지 않기 때문에 발생하는 에러**로 보인다. 따라서 그 부분을 수정해줘야 한다.

## 해결책

1. Ouster_ros 폴더 내의 **msg/PacketMsg.msg**내에 <code>std_msgs/Header header</code> 추가해야 함.

(수정후)

<pre><code>std_msgs/Header header</code>
<code>uint8[] buf</code></pre>

2. Ouster_ros 안의**src/os1_node.cpp**내에서 119번째 줄과 125번 줄에 

<pre><code>lidar_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>
<code>imu_packet.header.stamp.fromSec(ros::Time::now().toSec());</code></pre>

를 추가해줘야 한다.

원 파일의 일부분은 아래와 같다.


```cpp
int connection_loop(ros::NodeHandle& nh, OS1::client& cli) {
    auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(OS1::lidar_packet_bytes + 1);
    imu_packet.buf.resize(OS1::imu_packet_bytes + 1);

    while (ros::ok()) {
        auto state = OS1::poll_client(cli);
        if (state == OS1::EXIT) {
            ROS_INFO("poll_client: caught signal, exiting");
            return EXIT_SUCCESS;
        }
        if (state & OS1::ERROR) {
            ROS_ERROR("poll_client: returned error");
            return EXIT_FAILURE;
        }
        if (state & OS1::LIDAR_DATA) {
            if (OS1::read_lidar_packet(cli, lidar_packet.buf.data()))
            {
              lidar_packet_pub.publish(lidar_packet);
            }
        }
        if (state & OS1::IMU_DATA) {
            if (OS1::read_imu_packet(cli, imu_packet.buf.data()))
            {
              imu_packet_pub.publish(imu_packet);
            }
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
```

여기서 아래와 같이 119번째 줄과 125번 줄에 두 줄을 추가해주면 문제를 해결할 수 있다.

```cpp
int connection_loop(ros::NodeHandle& nh, OS1::client& cli) {
    auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

    PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(OS1::lidar_packet_bytes + 1);
    imu_packet.buf.resize(OS1::imu_packet_bytes + 1);

    while (ros::ok()) {
        auto state = OS1::poll_client(cli);
        if (state == OS1::EXIT) {
            ROS_INFO("poll_client: caught signal, exiting");
            return EXIT_SUCCESS;
        }
        if (state & OS1::ERROR) {
            ROS_ERROR("poll_client: returned error");
            return EXIT_FAILURE;
        }
        if (state & OS1::LIDAR_DATA) {
            if (OS1::read_lidar_packet(cli, lidar_packet.buf.data()))
            {
              lidar_packet.header.stamp.fromSec(ros::Time::now().toSec());
              lidar_packet_pub.publish(lidar_packet);
            }
        }
        if (state & OS1::IMU_DATA) {
            if (OS1::read_imu_packet(cli, imu_packet.buf.data()))
            {
              imu_packet.header.stamp.fromSec(ros::Time::now().toSec());
              imu_packet_pub.publish(imu_packet);
            }
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
```

## 결과

수정 결과, driver를 launch로 실행시켰을 때 아래와 같이 패킷이 **/os1_cloud_node/points**과 **/os1_cloud_node/imu**로 잘 파싱되는 것을 확인할 수 있다.

![ouster_after](/readme_materials/ouster_after.png)

sensor_smgs/PointCloud2로 잘 파싱되어 있는 것을 확인할 수 있고, 바로 rviz를 통해서도 데이터를 육안으로 확인이 가능하다. 

~~(새벽 한 시에 수정하고 데이터를 얻었어서...눈갱 죄송합니다)~~

![ouster_type](/readme_materials/ouster_type.png)

![ouster_data](/readme_materials/ouster-data.gif)
