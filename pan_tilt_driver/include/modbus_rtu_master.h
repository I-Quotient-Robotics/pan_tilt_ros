#ifndef MODBUSRTUMASTER_H
#define MODBUSRTUMASTER_H

#include <iostream>
#include <thread>
#include <string>
#include <mutex>

//OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <serial/serial.h>

class ModbusRTUMaster
{
public:
  ModbusRTUMaster(const std::string portNmae, const uint32_t baudRate);
  ~ModbusRTUMaster();

  uint8_t SetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data); //0x10 = 16
  uint8_t GetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data); //0x03 = 03

protected:
  static uint16_t ModBusCRC(const uint8_t *data, const uint8_t length);

private:
  std::mutex mtx_;
  serial::Serial com_;
};

#endif // !MODBUSRTUMASTER_H