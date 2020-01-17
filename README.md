# 3D LiDAR Graph SLAM 튜토리얼

연구실 내에 튜토리얼 자료로 구축 중...

아직 작성하지 않았습니다 

## WBU

## Description of rosbag file

* Mode: 



자칼의 xyz가 기존의 로봇들과 좀 다름!!! 거꾸로 가는듯한 느낌이 있음


## rosbag

<pre><code>rosbag play A4.bag --clock</code></pre>

<pre><code>rosparam set use_sim_time true</code></pre>

위의 두 명령어는 ros 상의 시간을 rosbag 기준으로 맞춰준다.

## launch ouster

<pre><code>roslaunch /home/shapelim/catkin_ws/src/ouster_example-master/ouster_ros/os1.launch replay:=true</code></pre>

**replay:=true** bag 파일의 패킷을 풀 때 필요함!

## When voxelizing
leaf size is too small!!

-> 0.2 이상으로 키우면 된다.


icp: target의 관점에서 source를 바라볼 때의 tf가 계산됨!
