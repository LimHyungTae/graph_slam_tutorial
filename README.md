# graph slam tutorial

## Ouster Settings

Ouster의 경우 센서 자체에서 TimeStamp를 찍어주지 않아서 그 부분을 추가해줘야 함!

* Ouster_ros 안의msg/PacketMsg.msg내에 <code>Header header</code> 추가해야 함.
* Ouster_ros 안의msg/os1_node.cpp내에 

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
아래 header의 stamp를 찍어주는 2줄 추가해줘야 함!
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
