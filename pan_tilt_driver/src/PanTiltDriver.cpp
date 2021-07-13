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
#include "PanTiltDriver.h"

static inline void delay(const uint32_t ms)
{
#if defined(_WIN32)
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

IQR::PanTiltDriver::PanTiltDriver(const uint8_t id, const std::string &portName)
    : id_(id), readFlage_(true)
{
  st_.id = 0;
  master_ = new ModbusRTUMaster(portName, 115200);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  setPose(0.0, 0.0, 10);
  start();
}

IQR::PanTiltDriver::~PanTiltDriver()
{
  stop();
  if (master_)
  {
    delete master_;
    master_ = NULL;
  }
}

void IQR::PanTiltDriver::getPose(float &yaw, float &pitch)
{
  std::lock_guard<std::mutex> lck(mtx_);
  yaw = st_.yaw_now;
  pitch = st_.pitch_now;
}

void IQR::PanTiltDriver::setPose(const float &yaw, const float &pitch, const uint16_t &speed)
{
  std::lock_guard<std::mutex> lck(mtx_);
  uint16_t sendBuf[3] = {
      0,
  };
  sendBuf[0] = speed;
  sendBuf[1] = int16_t(yaw*100.0);
  sendBuf[2] = int16_t(pitch*100.0);
  
  master_->SetMultipleRegisters(id_, 0x0006, 3, sendBuf);
}

void IQR::PanTiltDriver::run()
{
  uint16_t rcvdBuf[20] = {
      0,
  };
  while (readFlage_)
  {
    mtx_.lock();
    uint8_t rc = master_->GetMultipleRegisters(id_, 0x0000, 20, rcvdBuf);
    if (rc)
    {
      st_.id = rcvdBuf[0];
      st_.serial_num = str(boost::format("SN%1%") % int(rcvdBuf[1]));
      st_.hw_version = str(boost::format("v%1%.%2%") % int((rcvdBuf[2] & 0xff00) >> 8) % int(rcvdBuf[2] & 0x00ff));
      st_.bd_version = str(boost::format("v%1%.%2%") % int((rcvdBuf[3] & 0xff00) >> 8) % int(rcvdBuf[3] & 0x00ff));
      st_.sw_version = str(boost::format("v%1%.%2%.%3%") % int((rcvdBuf[4] & 0xf000) >> 12) % int((rcvdBuf[4] & 0x0f00) >> 8) % int(rcvdBuf[4] & 0x00ff));
      st_.set_zero = rcvdBuf[5];
      st_.speed = rcvdBuf[6];
      st_.yaw_goal = int16_t(rcvdBuf[7]) / 100.0;
      st_.pitch_goal = int16_t(rcvdBuf[8]) / 100.0;
      st_.reserved = rcvdBuf[9];
      st_.driver_ec = rcvdBuf[10];
      st_.encoder_ec = rcvdBuf[11];
      st_.yaw_now = int16_t(rcvdBuf[12]) / 100.0;
      st_.pitch_now = int16_t(rcvdBuf[13]) /100.0;
      st_.yaw_temp = int16_t(rcvdBuf[14]) / 10.0;
      st_.pitch_temp = int16_t(rcvdBuf[15]) /10.0;
      st_.yaw_raw = int16_t(rcvdBuf[16]);
      st_.pitch_raw = int16_t(rcvdBuf[17]);
      st_.loop_ec = rcvdBuf[18];
      st_.loop_time = rcvdBuf[19];
    }
    mtx_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

inline void IQR::PanTiltDriver::stop()
{
  std::lock_guard<std::mutex> lck(mtx_);
  readFlage_ = false;
}
