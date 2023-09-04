#include "modbus_rtu_master.h"

#include "rclcpp/rclcpp.hpp"


#define REBACK_SLEEP_MS 5

ModbusRTUMaster::ModbusRTUMaster(const std::string portNmae, const uint32_t baudRate) {
  try {
    com_.setPort(portNmae);
    com_.setBaudrate(baudRate);
    com_.setBytesize(serial::eightbits);
    com_.setParity(serial::parity_none);
    com_.setStopbits(serial::stopbits_one);
    com_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(50);
    com_.setTimeout(serial_timeout);
    com_.open();
    //com_.setRTS(false);
    //com_.setDTR(false);
  } catch (serial::IOException &e) {
    // RCLCPP_DEBUG(this->get_logger(),"Unable to open serial port:%s" , portNmae.c_str());
    return;
  }

  // RCLCPP_DEBUG(this->get_logger(),"open serial port:%s successful!!" , portNmae.c_str());
    
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

ModbusRTUMaster::~ModbusRTUMaster() {
  com_.close();
}

uint8_t ModbusRTUMaster::SetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data) {
  if (!com_.isOpen()) {
    // RCLCPP_DEBUG(this->get_logger(),"serial not open!!");
    return 0;
  }

  std::lock_guard<std::mutex> lck(mtx_);

  com_.flush();

  const uint8_t function_num = 0x10;
  uint8_t send_buffer[256] = {0};

  send_buffer[0] = slave_id;
  send_buffer[1] = function_num;
  send_buffer[2] = uint8_t((address & 0xff00) >> 8);
  send_buffer[3] = uint8_t((address & 0x00ff));
  send_buffer[4] = uint8_t((length & 0xff00) >> 8);
  send_buffer[5] = uint8_t((length & 0x00ff));
  send_buffer[6] = uint8_t((length & 0x00ff) * 2);
  uint8_t buffer_index = 6;
  for (uint16_t i = 0; i < length; i++) {
    send_buffer[7 + i * 2] = uint8_t((data[i] & 0xff00) >> 8);
    send_buffer[8 + i * 2] = uint8_t((data[i] & 0x00ff));
    buffer_index += 2;
  }

  uint16_t crc = ModBusCRC(send_buffer, buffer_index + 1);
  send_buffer[buffer_index + 1] = uint8_t((crc & 0x00ff));
  send_buffer[buffer_index + 2] = uint8_t((crc & 0xff00) >> 8);

  size_t buffer_length = buffer_index + 3;
  size_t write_length = com_.write(send_buffer, buffer_length);
  if (buffer_length != write_length) {
    // RCLCPP_DEBUG(this->get_logger(),"Failed to send message!!");
    com_.flush();
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(REBACK_SLEEP_MS));

  uint8_t receive_buffer[256] = {0};
  size_t receive_length = 8;

  size_t read_length = com_.read(receive_buffer, receive_length);

  if (receive_length != read_length) {
    // RCLCPP_DEBUG(this->get_logger(),"Failed to read message!!");
    com_.flush();
    return 0;
  }
  if (receive_buffer[0] != slave_id) {
    // RCLCPP_DEBUG(this->get_logger(),"Message ID error!!");
    com_.flush();
    return 0;
  }
  if (receive_buffer[1] != function_num) {
    // RCLCPP_DEBUG(this->get_logger(),"Message fuction number error!!");
    com_.flush();
    return 0;
  }

  uint16_t temp_address = uint16_t((receive_buffer[2] << 8) | receive_buffer[3]);
  uint16_t temp_length = uint16_t((receive_buffer[4] << 8) | receive_buffer[5]);

  if (temp_address != address) {
    // RCLCPP_DEBUG(this->get_logger(),"Message start address error!!");
    com_.flush();
    return 0;
  }

  if (temp_length != length) {
    // RCLCPP_DEBUG(this->get_logger(),"Message read length error!!");
    com_.flush();
    return 0;
  }

  uint16_t rec_crc = ModBusCRC(receive_buffer, receive_length - 2);
  uint16_t rec_crc_ = uint16_t((receive_buffer[receive_length - 1] << 8) | (receive_buffer[receive_length - 2]));

  if (rec_crc != rec_crc_) {
    // RCLCPP_DEBUG(this->get_logger(),"Message crc check error!!");
    com_.flush();
    return 0;
  }

  com_.flush();
  return 1;
}

uint8_t ModbusRTUMaster::GetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data) {
  if (!com_.isOpen()) {
    // RCLCPP_DEBUG(this->get_logger(),"serial not open!!");
    return 0;
  }

  std::lock_guard<std::mutex> lck(mtx_);

  com_.flush();

  const uint8_t function_num = 0x03;
  uint8_t send_buffer[8] = {0};

  send_buffer[0] = slave_id;
  send_buffer[1] = function_num;
  send_buffer[2] = uint8_t((address & 0xff00) >> 8);
  send_buffer[3] = uint8_t((address & 0x00ff));
  send_buffer[4] = uint8_t((length & 0xff00) >> 8);
  send_buffer[5] = uint8_t((length & 0x00ff));
  uint16_t crc = ModBusCRC(send_buffer, 6);
  send_buffer[6] = uint8_t((crc & 0x00ff));
  send_buffer[7] = uint8_t((crc & 0xff00) >> 8);

  size_t write_length = com_.write(send_buffer, 8);

  if (write_length != 8) {
    // RCLCPP_DEBUG(this->get_logger(),"Failed to send message!!");
    com_.flush();
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(REBACK_SLEEP_MS));

  uint8_t receive_buffer[256] = {0};
  size_t receive_length = (length * 2) + 5;
  size_t read_length = com_.read(receive_buffer, receive_length);

  if (receive_length != read_length) {
    // RCLCPP_DEBUG(this->get_logger(),"Failed to read message!!");
    com_.flush();
    return 0;
  }

  if (receive_buffer[0] != slave_id) {
    // RCLCPP_DEBUG(this->get_logger(),"Message ID error!!");
    com_.flush();
    return 0;
  }

  if (receive_buffer[1] != function_num) {
    // RCLCPP_DEBUG(this->get_logger(),"Message fuction number error!!");
    com_.flush();
    return 0;
  }

  if (receive_buffer[2] != (length * 2)) {
    // RCLCPP_DEBUG(this->get_logger(),"Message data length error!!");
    com_.flush();
    return 0;
  }

  uint16_t rec_crc = ModBusCRC(receive_buffer, receive_length - 2);
  uint16_t rec_crc_ = uint16_t((receive_buffer[receive_length - 1] << 8) | (receive_buffer[receive_length - 2]));

  if (rec_crc != rec_crc_) {
    // RCLCPP_DEBUG(this->get_logger(),"Message crc check error!!");
    com_.flush();
    return 0;
  }

  for (uint16_t i = 0; i < length; i++) {
    data[i] = uint16_t((receive_buffer[3 + i * 2] << 8) | (receive_buffer[4 + i * 2]));
  }

  com_.flush();
  return 1;
}

uint16_t ModbusRTUMaster::ModBusCRC(const uint8_t *data, const uint8_t length) {
  uint8_t j;
  uint16_t crc;
  uint8_t len = length;
  crc = 0xFFFF;

  while (len--) {
    crc = crc ^ *data++;
    for (j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }

  return crc;
  //return (crc << 8 | crc >> 8);
}