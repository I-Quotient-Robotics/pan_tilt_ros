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
#ifndef PANTILTDRIVER_H
#define PANTILTDRIVER_H

#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <mutex>
#include <boost/format.hpp>
#include "QThread.h"
#include "modbus_rtu_master.h"

namespace IQR
{
  typedef struct PanTiltStatus
  {                         //offect
    uint8_t id;             //0
    std::string serial_num; //1
    std::string hw_version; //2
    std::string bd_version; //3
    std::string sw_version; //4
    uint8_t set_zero;       //5
    uint16_t speed;         //6
    float yaw_goal;         //7
    float pitch_goal;       //8
    uint16_t reserved;      //9
    uint16_t driver_ec;     //10
    uint16_t encoder_ec;    //11
    float yaw_now;          //12
    float pitch_now;        //13
    float yaw_temp;         //14
    float pitch_temp;       //15
    int16_t yaw_raw;        //16
    int16_t pitch_raw;      //17
    uint16_t loop_ec;       //18
    uint16_t loop_time;     //19
  } PanTiltStatus;

  class PanTiltDriver : public QThread
  {
  public:
    PanTiltDriver(const uint8_t id, const std::string &portName);
    ~PanTiltDriver();

    bool getStatus(PanTiltStatus &st) { std::lock_guard<std::mutex> lck(mtx_); st = st_; return st_.id; };
    void getPose(float &yaw, float &pitch);
    void setPose(const float &yaw, const float &pitch, const uint16_t &speed);

  protected:
    void run();
    inline void stop();

  private:
    const uint8_t id_;
    bool readFlage_;
    std::mutex mtx_;
    PanTiltStatus st_;

    ModbusRTUMaster *master_;
  };
} // namespace IQR

#endif //PANTILTDRIVER_H
