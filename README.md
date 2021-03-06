# pan_tilt_ros
IQR第四代云台ROS驱动，[在线手册](http://doc.iquotient-robotics.com/pan_tilt_unit_user_manual/)。

## 环境要求
- Ubuntu 18.04
- ROS melodic

## 安装与编译
更新软件
```shell
sudo apt-get update
```
安装依赖项
```shell
sudo apt-get install ros-melodic-joy
```
创建ROS工作空间
```shell
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
克隆代码并编译
```shell
git clone https://github.com/I-Quotient-Robotics/pan_tilt_ros.git
cd ..
catkin_make
```
环境设置
```shell
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
添加串口rules文件
```shell
roscd pan_tilt_bringup/config/
sudo cp ./56-pan-tilt.rules /etc/udev/rules.d/
```
**注意：添加串口rules文件后，需要重新插拔USB线使之生效。**

## 使用
首先，连接好云台的电源和USB通信端口。
### 1.普通模式
启动驱动节点
```shell
roslaunch pan_tilt_bringup panTilt_view.launch
```
发送位置消息
```shell
rostopic pub /pan_tilt_cmd_deg pan_tilt_msgs/PanTiltCmdDeg "yaw: 30.0
pitch: 30.0
speed: 10" 
```

### 2.手柄控制模式
连接好手柄，然后启动驱动节点
```shell
roslaunch pan_tilt_bringup panTilt_view.launch use_joy:=true
```
使用手柄上的 *X* *Y* *A* *B*按键来控制云台的俯仰和旋转。
