# navigator

## 话题
|名称|类型|发布者|说明|
|---|----|-----|---|
|`/remote_cmd`|远程控制命令，其中包括了**vehicle_cmd**相关的数据|
|`/twist_cmd`|当前车的速度与方位信息|
|`/mode_cmd`|车辆模式命令|
|`/gear_cmd`|车辆档位信息|
|`/accel_cmd`|加速度信息|
|`/steer_cmd`|转向值|
|`/break_cmd`|刹车值|
|`/lamp_cmd`|车灯信息|
|`/ctrl_cmd`|控制命令|

### 关于车辆控制话题精讲解

## runtime_manager

> rosrun runtime\_manager runtime\_manager

## flir

> rosrun flir spinnaker

$\quad$启动_flir_的相关摄像头的驱动。

## velodyne_pointcloud

$\quad$此包负责启动关于_velodyne_激光雷达的相关功能。

> roslaunch velodyne launch\_file

| name                          | describe |
|-------------------------------|----------|
| *velodyne_hdl32c.launch*      |          |
| *velodyne_vlp16.launch*       |          |
| *velodyne_hdl32e.launch*      |          |
| *velodyne_hdl64e_s2.launch*   |          |
| *VLP-32C_points.launch*       |          |
| *VLP16_points.launch*         |          |
| *velodyne_hdl64e_s3.launch*   |          |
| *velodyne_vlp16_hires.launch* |          |
| *velodyne_vlp32c.launch*      |          |
| *32c_points.launch*           |          |
| *32e_points.launch*           |          |
| *cloud_nodelet.launch*        |          |
| *laserscan_nodelet.launch*    |          |
| *transform_nodelet.launch*    |          |

## lidar_localizer
$\quad$此包的负责

## decision_maker

### decision_maker

### planner_selector

## waypoint_follower
$\quad$在此包中存在四个节点。其中**twist\_gate**与**twist\_filter**两个节点是一同使用的。通过_launch_进行一起进行启动。

### twist_gate

> roslaunch waypoint\_follower twist\_filter.launch

$\quad$这个节点会定于订阅车辆状态的所有信息，接到一个话题后将其设置到`vehicle_cmd`并将其发送，从代码中可以看到的是，此节点并不是全部将`vehicle_cmd`所需的字段数值提取到后发送，而是接收到一部分数据后便发送一部分数据到`vehicle_cmd`话题。这个话题是由
**vehicle\_sender**发布的，发送节点会合成一个字符串数据发送给**CAN卡**客户端。

#### 订阅的_topic_列表

|名称|说明|
|---|----|
|`/remote_cmd`|远程控制命令，其中包括了**vehicle_cmd**相关的数据|
|`/twist_cmd`|当前车的速度与方位信息|
|`/mode_cmd`|车辆模式命令|
|`/gear_cmd`|车辆档位信息|
|`/accel_cmd`|加速度信息|
|`/steer_cmd`|转向值|
|`/break_cmd`|刹车值|
|`/lamp_cmd`|车灯信息|
|`/ctrl_cmd`|控制命令|

#### 发布的_topicn_列表

|名称|说明|
|---|----|
|`/emergency_stop`|发布一个立即停止的标志|
|`/ctrl_mode`|一个字符串，标记当前的控制类型是自动控制，还是人进行远程控制,"_AUTO_"或者“_REMOTE_”或者"_UNDEFINED_"|
|`/state_cmd`|车辆状态|
|`/vehicle_cmd`|`车辆控制命令|

### twist_filter

> roslaunch waypoint\_follower twist\_filter.launch

$\quad$此节点与**twist_gate**节点配合使用，此节点订阅了,`twist_raw`与`config/twist_filter`两个话题，通过`config/twist_filter`的配置信息，使用`twist_raw`的原始数据来计算出提供给`twist_gate`使用的`twist_cmd`话题信息。

$\quad$**config/twist_filter**话题是三个配置信息。

* `lateral_accel_limit`         横向加速度的最高限制
* `lowpass_gain_linear_x`       使用当前线性速度的百分比
* `lowpass_gain_angular_z`      使用当前转角的百分比

这里节点目的是限制横向加速度过大，加强驾驶安全。从原始车辆信息接收到信息后，通过此节点进行安全过滤。最终将过滤好的数据发布到`twist_cmd`。

#### 过滤算法

其代码如下：
```c++
void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg) {
  double v = msg->twist.linear.x;
  double omega = msg->twist.angular.z;

  if(fabs(omega) < ERROR){
    g_twist_pub.publish(*msg);
    return;
  }

  double max_v = g_lateral_accel_limit / omega;

  geometry_msgs::TwistStamped tp;
  tp.header = msg->header;

  double a = v * omega;
  ROS_INFO("lateral accel = %lf", a);


  tp.twist.linear.x = fabs(a) > g_lateral_accel_limit ? max_v
                    : v;
  tp.twist.angular.z = omega;

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  lowpass_linear_x = g_lowpass_gain_linear_x * lowpass_linear_x + 
                (1 - g_lowpass_gain_linear_x) * tp.twist.linear.x;
  lowpass_angular_z = g_lowpass_gain_angular_z * lowpass_angular_z + 
                (1 - g_lowpass_gain_angular_z) * tp.twist.angular.z;

  tp.twist.linear.x = lowpass_linear_x;
  tp.twist.angular.z = lowpass_angular_z;

  ROS_INFO("v: %f -> %f",v,tp.twist.linear.x);
  g_twist_pub.publish(tp);
}
```

### pure_pursuit

> roslaunch waypoint\_follower pure\_pursuit.launch

$\quad$此节点是此包内的最重要的节点，负责对路点进行**插值运算**。此节点的启动必须在接收到四个订阅的话题后才能启动。

* 位置信息，通过话题`config/waypoint_follower`接收。
* 路点信息，通过话题`final_waypoints`接收。
* 速度信息，通过话题`current_velocity`接收。
* 配置信息，通过话题`current_pose`接收。

此节点最终要的功能就是从`final_waypoints`话题订阅来的最终路点并通过**插值算法**将其生成一条平滑的曲线。并按照

$\quad$此节点还负责发布一些对**Rviz**可视化的支持，例如：显示下一个路点、显示搜索半径、显示下一个目标点以及显示轨迹等。

#### 订阅的_topic_列表

|名称|说明|
|---|----|
|`/config/waypoint_follower`|当前配置信息|
|`/final_waypoints`|最终的路点信息|
|`/current_pose`|车辆当前位置|
|`/current_velocity`|车辆当前速度|

#### 发布的_topic_列表

|名称|说明|
|---|----|
|`/twist_raw`|远程控制命令，其中包括了**vehicle_cmd**相关的数据|
|`/ctrl_cmd`|当前车的速度与方位信息|

#### 发布的_topic_列表，对_Rviz_可视化提供数据
|`/next_waypoint_mark`|下一个路点的标记|
|`/next_target_mark`|下一个目标标记|
|`/search_circle_mark`|搜索圆标记|
|`/line_point_mark`|线点标记|
|`/trajectory_circle_mark`|轨迹标记|
|`/angular_gravity`|角度|
|`/deviation_of_current_position`|当前位置的导数|

### wf_simulator

> roslaunch waypoint\_follower wf\_simulator.launch

## vehicle_socket
$\quad$此包是无人驾驶系统驱动车辆的最后一道关口，负责从CAN驱动读取数据与发送数据到CAN驱动。其总两个节点一个负责接收一个负责发送。两个节点是两个**TCP**服务器。CAN驱动是一个客户端程序负责链接到这两个服务器程序来交换数据。

### vehicle_receiver

> rosrun vehicle\_socket vehicle\_receiver

### vehicle_sender

> rosrun vehicle\_socket vehicle\_sender

## CAN卡驱动
