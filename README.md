# graph slam tutorial

## Description of rosbag file

* Mode: 

WBU

자칼의 xyz가 이상함!!! 거꾸로 가는듯한 느낌이 있음


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
