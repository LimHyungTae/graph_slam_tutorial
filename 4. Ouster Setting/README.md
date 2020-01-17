# Setting of Ouster OS1-64

## 문제가 발생하는 이유?

Ouster 사에서 제공하는 driver내에서 msg 상에 header가 포함되어 있지 않고, 그로 인해 ROS 상에서 data를 받게 되면 error가 발생한다. 

다시 말하자면, 센서의 드라이버 내에서 TimeStamp를 찍어주지 않기 때문에 그 부분을 수정해줘야 함!

## Solution

<code>catkin make ouster_ros</code>

1. Ouster 사에서 제공하는 driver를 git clone한다. (2020-01-09 기준)

<code>git clone https://github.com/ouster-lidar/ouster_example</code>

2. Ouster_ros 안의msg/PacketMsg.msg내에 <code>Header header</code> 추가해야 함.

3. Ouster_ros 안의msg/os1_node.cpp내에서  

<code>lidar_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>

<code>imu_packet.header.stamp.fromSec(ros::Time::now().toSec());</code>

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
