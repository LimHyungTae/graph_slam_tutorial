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
    
**그런데...막상 launch file을 실행하면 아래와 같이 문제가 발생한다!!!!**
    
![error_occurs](/readme_materials/ouster_error.png)    
    
## 문제가 발생하는 이유?

Ouster 사에서 제공하는 driver내에서 msg 상에 header가 포함되어 있지 않고, 그로 인해 ROS 상에서 data를 받게 되면 error가 발생한다. 

다시 말하자면, 센서의 드라이버 내에서 TimeStamp를 찍어주지 않기 때문에 그 부분을 수정해줘야 함!

## Solution

1. 
2. Ouster_ros 안의msg/PacketMsg.msg내에 <code>Header header</code> 추가해야 함.

3. Ouster_ros 안의msg/os1_node.cpp내에서  

<code>lidar_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>

<code>imu_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>

를 추가해줘야 한다.

원 파일의 일부분은 아래와 같다.



<code>catkin make ouster_ros</code>

1. Ouster 사에서 제공하는 driver를 git clone한다. (2020-01-09 기준)

<code>git clone https://github.com/ouster-lidar/ouster_example</code>

2. Ouster_ros 안의msg/PacketMsg.msg내에 <code>Header header</code> 추가해야 함.

3. Ouster_ros 안의src/os1_node.cpp내에서 119번째 줄과 125번 줄에  

<code>lidar_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>

<code>imu_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>

를 추가해줘야 한다.


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
아래와 같이 두 줄을 추가해주면 데이터를 받는데에 문제를 해결할 수 있다.
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
