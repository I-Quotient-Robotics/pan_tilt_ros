# pan_tilt_ros

> 警告 Warning
>
> This ROS2 package is beta version.



IQR第四代云台ROS2驱动，[在线手册](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## 环境要求
- Ubuntu 22.04
- ROS Humble

## 安装与编译
创建ROS工作空间
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
克隆代码并编译
```shell
git clone -b humble-devel https://github.com/I-Quotient-Robotics/pan_tilt_ros.git
cd ..
colcon build --symlink-install
```
环境设置
```shell
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
添加串口rules文件
```shell
cd ros2_ws/src/pan_tilt_ros/pan_tilt_bringup/config/
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```
**注意：添加串口rules文件后，需要重新插拔USB线使之生效。**

## 使用
1. 首先，连接好云台的电源和USB通信端口。

2. 启动驱动节点

    ```shell
    ros2 launch pan_tilt_bringup panTilt_bringup.launch.py
	```