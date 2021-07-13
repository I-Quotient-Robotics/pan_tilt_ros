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
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "pan_tilt_msgs/PanTiltCmdDeg.h"
#include "pan_tilt_msgs/PanTiltCmdRad.h"
#include "pan_tilt_msgs/PanTiltStatus.h"
#include "PanTiltDriver.h"

using namespace std;

class PanTiltDriverNode
{
public:
  PanTiltDriverNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~PanTiltDriverNode();
  void JointStatePublish();

protected:
  void callBackDeg(const pan_tilt_msgs::PanTiltCmdDeg &msg);
  void callBackRad(const pan_tilt_msgs::PanTiltCmdRad &msg);

private:
  IQR::PanTiltDriver *pt_;
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber cmdDegSub_;
  ros::Subscriber cmdRadSub_;
  ros::Publisher jointPub_;
  ros::Publisher statusPub_;
  int id_;
  std::string portName_;
  std::string yawJointName_;
  std::string pitchJointName_;
  sensor_msgs::JointState js_;
};

PanTiltDriverNode::PanTiltDriverNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  //init params
  private_nh_.param<int>("ID", id_, 0x01);
  private_nh_.param<std::string>("port_name", portName_, "/dev/ttyUSB0");
  private_nh_.param<std::string>("yaw_joint_name", yawJointName_, "iqr_pan_tilt_yaw_joint");
  private_nh_.param<std::string>("pitch_joint_name", pitchJointName_, "iqr_pan_tilt_pitch_joint");

  js_.name.resize(2);
  js_.position.resize(2);
  js_.velocity.resize(2);
  js_.effort.resize(2);
  js_.name[0] = yawJointName_;
  js_.name[1] = pitchJointName_;

  //set publisher
  jointPub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  statusPub_ = nh_.advertise<pan_tilt_msgs::PanTiltStatus>("/pan_tilt_status", 10);

  //set subscriber
  cmdDegSub_ = nh_.subscribe("pan_tilt_cmd_deg", 10, &PanTiltDriverNode::callBackDeg, this);
  cmdRadSub_ = nh_.subscribe("pan_tilt_cmd_rad", 10, &PanTiltDriverNode::callBackRad, this);

  cout << "pan-tilt ID:" << id_ << " ,port name:" << portName_ << endl;
  pt_ = new IQR::PanTiltDriver(id_, portName_);
}

PanTiltDriverNode::~PanTiltDriverNode()
{
  if (pt_)
  {
    delete pt_;
    pt_ = NULL;
  }
}

void PanTiltDriverNode::JointStatePublish()
{
  IQR::PanTiltStatus pt_st;
  if (pt_->getStatus(pt_st))
  {
    double yawRad = 0.01745329 * pt_st.yaw_now;
    double pitchRad = 0.01745329 * pt_st.pitch_now;

    js_.header.stamp = ros::Time::now();
    js_.position[0] = yawRad;
    js_.position[1] = pitchRad;
    jointPub_.publish(js_);

    pan_tilt_msgs::PanTiltStatus st;
    st.header.stamp = ros::Time::now();
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
    statusPub_.publish(st);
  }
}

void PanTiltDriverNode::callBackRad(const pan_tilt_msgs::PanTiltCmdRad &msg)
{
  if (msg.yaw > 1.0471974 || msg.yaw < -1.0471974 
    || msg.pitch > 1.0471974 || msg.pitch < -1.0471974 
    || msg.speed > 0.5235987 || msg.speed < 0.01745329)
  {
    ROS_WARN_STREAM("Input param error, yaw:[-1.0471974, 1.0471974] pitch:[-1.0471974, 1.0471974] speed[0.01745329, 0.5235987]");
    return;
  }

  pt_->setPose(float(msg.yaw/0.01745329), float(msg.pitch/0.01745329), uint16_t(msg.speed/0.01745329));
}

void PanTiltDriverNode::callBackDeg(const pan_tilt_msgs::PanTiltCmdDeg &msg)
{
  // ROS_INFO("[%f,%f,%i]", msg.yaw, msg.pitch, msg.speed);
  if (msg.yaw > 60.0 || msg.yaw < -60.0 
    || msg.pitch > 60.0 || msg.pitch < -60.0 
    || msg.speed > 30 || msg.speed <= 0)
  {
    ROS_WARN_STREAM("Input param error, yaw:[-60, 60] pitch:[-60, 60] speed[1, 30]");
    return;
  }

  pt_->setPose(msg.yaw, msg.pitch, msg.speed);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "PanTiltDriverNode");

  ros::NodeHandle nh, privatenh_("~");

  PanTiltDriverNode nd(nh, privatenh_);
  ros::Rate r(50);

  while (ros::ok())
  {
    ros::spinOnce();
    nd.JointStatePublish();
    r.sleep();
  }

  ROS_INFO_STREAM("Finish!!");
  return 0;
}
