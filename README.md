# navigator

## runtime_manager

> rosrun runtime\_manager runtime\_manager

## flir

> rosrun flir spinnaker

## velodyne_pointcloud

| name                          | describe |
|-------------------------------|----------|
| *32c_points.launch*           |          |
| *velodyne_hdl32c.launch*      |          |
| *velodyne_vlp16.launch*       |          |
| *32e_points.launch*           |          |
| *velodyne_hdl32e.launch*      |          |
| *velodyne_vlp32c.launch*      |          |
| *cloud_nodelet.launch*        |          |
| *velodyne_hdl64e_s2.launch*   |          |
| *VLP16_points.launch*         |          |
| *laserscan_nodelet.launch*    |          |
| *velodyne_hdl64e_s3.launch*   |          |
| *VLP-32C_points.launch*       |          |
| *transform_nodelet.launch*    |          |
| *velodyne_vlp16_hires.launch* |          |

> roslaunch velodyne launch\_file

## lidar_localizer

## waypoint_follower

### pure_pursuit

### twist_filter

### twist_gate

#### 发布的_topicn_列表

|名称|说明|
|---|----|
|/emergency\_stop||
|/ctrl\_mode||
|/state\_cmd||
|/vehicle\_cmd||

#### 订阅的_topic_列表

|名称|说明|
|---|----|
|/twist\_cmd||
|/mode\_cmd||
|/gear\_cmd||
|/accel\_cmd||
|/steer\_cmd||
|/break\_cmd||
|/lamp\_cmd||
|/ctrl\_cmd||

### wf_simulator

## vehicle_socket

### vehicle_receiver

> rosrun vehicle\_socket vehicle\_receiver

### vehicle_sender

> rosrun vehicle\_socket vehicle\_sender
