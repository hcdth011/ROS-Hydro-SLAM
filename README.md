# ROS-Hydro-SLAM
几种主要的SLAM方法（Libviso,SVO,PTAM）在ROS-Hydro平台下的实现，以及为实现实时视觉SLAM,添加了USB摄像头节点usb_cam。

Libviso2调试方法：
1. 通过一个launch文件运行所有ROS节点：
   roscore
   roslaunch viso2_ros mono.launch

2. 进入RVIZ，调整参数
   a. 将Fixed Frame改为 odom
   b. "Add"新的"display" : 一个Odomety，选上对应的topic ；
                           一个 Pose ,选上对应的topic，为了跟odometry区分，可以改变他的color ；
                           一个Image，选上topic (image_rect);

3. 移动相机

SVO调试方法：
1.查看USB摄像头信息
   lsusb -v或者lsusb  来查看USB摄像头的idVender和idProduct ；
   或者在/dev/目录下可以查到videoX设备号

2. 安装ROS下的usb摄像头驱动包(usb_cam)
   https://github.com/bosch-ros-pkg/usb_cam
   http://wiki.ros.org/usb_cam
   把包下载到catkin_ws/src下即可，然后catkin_make编译。

   默认使用的摄像头设备是video0 ，如果需要换其他设备 ，修改usb_cam/nodes下的usb_cam_node.cpp文件===》 std::string("/dev/video1")
   （注意：如果出现‘VIDIOC_S_FMT error 22, Invalid argument’的报错，即摄像头序号不对，修改video的号码重新编译即可）
   修改cpp后需要重新编译；
   运行方法： rosrun usb_cam usb_cam_node  ,运行后生成的图片topic为 /usb_cam/image_raw （svo_ros的launch文件需要修改对应的主题名字）

3. run rviz rviz报错：compiled against OGRE version 1.7.4 (Cthugha)，Stereo is N OT SUPPORTED，OpenGl version: 2.1 (GLSL 1.2).


   方法1：显卡驱动没装（安装驱动），或者显卡不支持硬件加速 ==》 到系统设置里面安装附加驱动，重启，确认显卡驱动“激活并在使用”；
  
   方法2：对于有双显卡的电脑（比如恶心的联想），系统自带的驱动安装方法可能不管用，可尝试安装 Bumblebee ,方法如下： 
         https://wiki.ubuntu.com/Bumblebee#Installation

   安装完驱动后，重新运行 run rviz rviz


4. 摄像头标定(pinhole模型)
   http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

   roscore
   rosrun usb_cam usb_cam_node
   rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/usb_cam/image_raw camera:=/usb_cam

5. 摄像头标定(ATAN模型)
   安装ethzasl_ptam包：  http://wiki.ros.org/ethzasl_ptam
   标定方法： http://wiki.ros.org/ethzasl_ptam/Tutorials/camera_calibration

   roscore
   roslaunch ptam cameracalibrator.launch （launch文件已经被修改）
   
6. svo测试方法：
  （1）Run SVO on a Dataset

   roscore
   roslaunch svo_ros test_rig3.launch
   rosrun rviz rviz -d ~/catkin_ws/src/rpg_svo/svo_ros/rviz_config.rviz
   rosbag play ~/svo_workspace/airground_rig_s3_2013-03-18_21-38-48.bag

   (2)Run SVO on a live camera stream

   事前准备：摄像头标定（参数保存为my_camera_pinhole.yaml）,修改live.launch文件（使用/usb_cam/image_raw和my_camera_pinhole.yaml）。

   roscore
   rosrun usb_cam usb_cam_node
   roslaunch svo_ros live.launch
   rosrun rqt_svo rqt_svo

   查看姿态数据：rostopic echo /svo/pose



