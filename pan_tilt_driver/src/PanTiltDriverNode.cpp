/**********************************************************************************
** MIT License
** 
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** 
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
// #include <ros/time.h>
#include "pan_tilt_msgs/msg/pan_joint_state.hpp"
// #include "pan_tilt_msgs/msg/PanTiltJointstate.h"
#include <tf2_ros/transform_broadcaster.h>
#include "pan_tilt_msgs/msg/pan_tilt_cmd_deg.hpp"
#include "pan_tilt_msgs/msg/pan_tilt_cmd_rad.hpp"
#include "pan_tilt_msgs/msg/pan_tilt_status.hpp"
#include "PanTiltDriver.h"
#include "std_msgs/msg/string.hpp"

using namespace std;
using std::placeholders::_1;
class PanTiltDriverNode:public rclcpp::Node
{
public:
  PanTiltDriverNode(std::string node_name): Node(node_name)
  {
    //init params
  this->declare_parameter<std::string>("port_name");
  this->declare_parameter<int>("ID");
  this->declare_parameter<std::string>("yaw_joint_name");
  this->declare_parameter<std::string>("pitch_joint_name");
  this->get_parameter_or<int>("ID", id_, 0x01);
  this->get_parameter_or<std::string>("port_name", portName_, "/dev/ttyUSB0");
  this->get_parameter_or<std::string>("yaw_joint_name", yawJointName_, "iqr_pan_tilt_yaw_joint");
  this->get_parameter_or<std::string>("pitch_joint_name", pitchJointName_, "iqr_pan_tilt_pitch_joint");

  // js_.name.resize(2);
  // js_.position.resize(2);
  // js_.velocity.resize(2);
  // js_.effort.resize(2);
  js_.name1 = yawJointName_;
  js_.name2 = pitchJointName_;

  //set publisher
  jointPub_ = this->create_publisher<pan_tilt_msgs::msg::PanJointState>("/joint_states", 10);
  statusPub_ = this->create_publisher<pan_tilt_msgs::msg::PanTiltStatus>("/pan_tilt_status", 10);

  //set subscriber
  cmdDegSub_ = this->create_subscription<pan_tilt_msgs::msg::PanTiltCmdDeg>("/pan_tilt_cmd_deg", 10, std::bind(&PanTiltDriverNode::callBackDeg, this,_1));
  cmdRadSub_ = this->create_subscription<pan_tilt_msgs::msg::PanTiltCmdRad>("/pan_tilt_cmd_rad", 10, std::bind(&PanTiltDriverNode::callBackRad, this,_1));
  
  cout << "pan-tilt ID:" << id_ << " ,port name:" << portName_ << endl;
  pt_ = new IQR::PanTiltDriver(id_, portName_);
  }
  ~PanTiltDriverNode()
  {
    if (pt_)
    {
      delete pt_;
      pt_ = NULL;
    }
  }
  void JointStatePublish();

protected:
  void callBackDeg(const pan_tilt_msgs::msg::PanTiltCmdDeg::SharedPtr msg);
  void callBackRad(const pan_tilt_msgs::msg::PanTiltCmdRad::SharedPtr msg);

private:
  IQR::PanTiltDriver *pt_;
  rclcpp::Node *node_;
  rclcpp::Subscription<pan_tilt_msgs::msg::PanTiltCmdDeg>::SharedPtr cmdDegSub_;
  rclcpp::Subscription< pan_tilt_msgs::msg::PanTiltCmdRad>::SharedPtr cmdRadSub_;
  rclcpp::Publisher<pan_tilt_msgs::msg::PanJointState>::SharedPtr jointPub_;
  rclcpp::Publisher<pan_tilt_msgs::msg::PanTiltStatus>::SharedPtr statusPub_;
  int id_;
  std::string portName_;
  std::string yawJointName_;
  std::string pitchJointName_;
  pan_tilt_msgs::msg::PanJointState js_;
};

// PanTiltDriverNode::PanTiltDriverNode(std::string node_name)
//     : Node(node_name)
// {
  
  
// }

// PanTiltDriverNode::~PanTiltDriverNode()
// {
//   if (pt_)
//   {
//     delete pt_;
//     pt_ = NULL;
//   }
// }

void PanTiltDriverNode::JointStatePublish()
{
  IQR::PanTiltStatus pt_st;
  if (pt_->getStatus(pt_st))
  {
    double yawRad = 0.01745329 * pt_st.yaw_now;
    double pitchRad = 0.01745329 * pt_st.pitch_now;

    js_.header.stamp = this->get_clock()->now();
    js_.position1 = yawRad;
    js_.position2 = pitchRad;
    jointPub_->publish(js_);

    pan_tilt_msgs::msg::PanTiltStatus st;
    st.header.stamp = this->get_clock()->now();
    st.id = pt_st.id;
    st.serial_num = pt_st.serial_num;
    st.hw_version = pt_st.hw_version;
    st.bd_version = pt_st.bd_version;
    st.sw_version = pt_st.sw_version;
    st.set_zero = pt_st.set_zero;
    st.speed = pt_st.speed;
    st.yaw_goal = pt_st.yaw_goal;
    st.pitch_goal = pt_st.pitch_goal;
    st.reserved = pt_st.reserved;
    st.driver_ec = pt_st.driver_ec;
    st.encoder_ec = pt_st.encoder_ec;
    st.yaw_now = pt_st.yaw_now;
    st.pitch_now = pt_st.pitch_now;
    st.yaw_temp = pt_st.yaw_temp;
    st.pitch_temp = pt_st.pitch_temp;
    st.yaw_raw = pt_st.yaw_raw;
    st.pitch_raw = pt_st.pitch_raw;
    st.loop_ec = pt_st.loop_ec;
    st.loop_time = pt_st.loop_time;
    statusPub_->publish(st);
  }
}

void PanTiltDriverNode::callBackRad(const pan_tilt_msgs::msg::PanTiltCmdRad::SharedPtr msg)
{
  if (msg->yaw > 1.0471974 || msg->yaw < -1.0471974 
    || msg->pitch > 1.0471974 || msg->pitch < -1.0471974 
    || msg->speed > 0.5235987 || msg->speed < 0.01745329)
  {
    RCLCPP_INFO(this->get_logger(),"Input param error, yaw:[-1.0471974, 1.0471974] pitch:[-1.0471974, 1.0471974] speed[0.01745329, 0.5235987]");
    return;
  }

  pt_->setPose(float(msg->yaw/0.01745329), float(msg->pitch/0.01745329), uint16_t(msg->speed/0.01745329));
}

void PanTiltDriverNode::callBackDeg(const pan_tilt_msgs::msg::PanTiltCmdDeg::SharedPtr msg)
{
  // ROS_INFO("[%f,%f,%i]", msg.yaw, msg.pitch, msg.speed);
  if (msg->yaw > 60.0 || msg->yaw < -60.0 
    || msg->pitch > 60.0 || msg->pitch < -60.0 
    || msg->speed > 30 || msg->speed <= 0)
  {
    RCLCPP_INFO(this->get_logger(),"Input param error, yaw:[-60, 60] pitch:[-60, 60] speed[1, 30]");
    return;
  }

  pt_->setPose(msg->yaw, msg->pitch, msg->speed);
}

int main(int argc, char *argv[])
{
  // ros::init(argc, argv, "PanTiltDriverNode");
  rclcpp::init(argc,argv);
  auto driver_node=std::make_shared<PanTiltDriverNode>("PanTiltDriverNode");
  rclcpp::WallRate rate(50);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(driver_node);
    driver_node->JointStatePublish();
    rate.sleep();
  }
  rclcpp::shutdown();
  // RCLCPP_INFO(this->get_logger(),"Finish!!");
  return 0;
}
