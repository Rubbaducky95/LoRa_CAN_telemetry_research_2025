#include "CAN_functions.h"
#include <Arduino_CAN.h>
#include <string.h>

// External reference to vehicle data
extern struct VehicleData {
  float motor_current;
  float motor_temp;
  float velocity;
  float distance_travelled;
  float battery_volt;
  float battery_current;
  float battery_cell_LOW_volt;
  float battery_cell_HIGH_volt;
  float battery_cell_AVG_volt;
  float battery_cell_LOW_temp;
  float battery_cell_HIGH_temp;
  float battery_cell_AVG_temp;
  uint8_t battery_cell_ID_HIGH_temp;
  uint8_t battery_cell_ID_LOW_temp;
  float BMS_temp;
  float MPPT1_input_voltage;
  float MPPT1_input_current;
  float MPPT1_output_voltage;
  float MPPT1_output_current;
  float MPPT1_power;
  float MPPT2_input_voltage;
  float MPPT2_input_current;
  float MPPT2_output_voltage;
  float MPPT2_output_current;
  float MPPT2_power;
  float MPPT3_input_voltage;
  float MPPT3_input_current;
  float MPPT3_output_voltage;
  float MPPT3_output_current;
  float MPPT3_power;
  float MPPT_total_watt;
} vehicleData;

bool setupCAN() {
  // Initialize CAN bus at 500 kbps (matching ESP32 configuration)
  // Note: Arduino R4 Minima uses fixed pins:
  //   D4 = CANTX0 (CAN Transmit) -> Connect to CAN transceiver TXD
  //   D5 = CANRX0 (CAN Receive)  -> Connect to CAN transceiver RXD
  // These pins are hardcoded and cannot be changed
  if (!CAN.begin(CanBitRate::BR_500k)) {
    return false;
  }
  
  Serial.println("CAN bus started at 500 kbps");
  Serial.println("CAN TX pin: D4 (CANTX0)");
  Serial.println("CAN RX pin: D5 (CANRX0)");
  
  return true;
}

void sendFloat(uint16_t canId, float value1, float value2) {
  // Pack two floats into 8 bytes (4 bytes each)
  CanMsg msg;
  msg.id = canId;
  msg.data_length = 8;
  
  // Convert floats to bytes (little-endian)
  memcpy(&msg.data[0], &value1, 4);
  memcpy(&msg.data[4], &value2, 4);
  
  CAN.write(msg);
}

void sendFloatAtOffset(uint16_t canId, float value, uint8_t byteOffset) {
  // Send a float at a specific byte offset in the CAN message
  CanMsg msg;
  msg.id = canId;
  msg.data_length = 8;
  
  // Initialize all bytes to 0
  memset(msg.data, 0, 8);
  
  // Copy float to specified offset (little-endian)
  memcpy(&msg.data[byteOffset], &value, 4);
  
  CAN.write(msg);
}

void sendUint8(uint16_t canId, uint8_t value, uint8_t byteOffset) {
  // Send a uint8 at a specific byte offset
  CanMsg msg;
  msg.id = canId;
  msg.data_length = 8;
  
  // Initialize all bytes to 0
  memset(msg.data, 0, 8);
  
  msg.data[byteOffset] = value;
  
  CAN.write(msg);
}

void sendInt16Scaled(uint16_t canId, int16_t value, uint8_t byteOffset) {
  // Send a signed 16-bit integer at a specific byte offset (little-endian)
  CanMsg msg;
  msg.id = canId;
  msg.data_length = 8;
  
  // Initialize all bytes to 0
  memset(msg.data, 0, 8);
  
  msg.data[byteOffset] = value & 0xFF;
  msg.data[byteOffset + 1] = (value >> 8) & 0xFF;
  
  CAN.write(msg);
}

void sendUint16Scaled(uint16_t canId, uint16_t value, uint8_t byteOffset) {
  // Send an unsigned 16-bit integer at a specific byte offset (little-endian)
  CanMsg msg;
  msg.id = canId;
  msg.data_length = 8;
  
  // Initialize all bytes to 0
  memset(msg.data, 0, 8);
  
  msg.data[byteOffset] = value & 0xFF;
  msg.data[byteOffset + 1] = (value >> 8) & 0xFF;
  
  CAN.write(msg);
}

void sendMotorData() {
  // CAN ID 0x402: Motor Current (bytes 4-7: float)
  sendFloatAtOffset(CAN_ID::MOTOR_CURRENT, vehicleData.motor_current, 4);
  
  // CAN ID 0x40B: Motor Temperature (bytes 0-3: float) + MC Temperature (bytes 4-7: float)
  sendFloat(CAN_ID::MOTOR_MC_TEMP, vehicleData.motor_temp, vehicleData.motor_temp); // Note: MC temp same as motor temp for now
}

void sendVehicleData() {
  // CAN ID 0x403: Velocity (bytes 4-7: float)
  sendFloatAtOffset(CAN_ID::VELOCITY, vehicleData.velocity, 4);
  
  // CAN ID 0x40E: Distance Travelled (bytes 0-3: float)
  sendFloatAtOffset(CAN_ID::DISTANCE_TRAVELLED, vehicleData.distance_travelled, 0);
}

void sendBatteryData() {
  // CAN ID 0x601: Battery temperatures and cell IDs
  // Byte 1: BMS Temperature (uint8)
  // Byte 2: High Cell Temperature (uint8)
  // Byte 3: Low Cell Temperature (uint8)
  // Byte 4: Avg Cell Temperature (uint8)
  // Byte 5: High Temp Cell ID (uint8)
  // Byte 6: Low Temp Cell ID (uint8)
  CanMsg msg;
  msg.id = CAN_ID::BATTERY_TEMPS;
  msg.data_length = 8;
  memset(msg.data, 0, 8);
  msg.data[1] = (uint8_t)vehicleData.BMS_temp;
  msg.data[2] = (uint8_t)vehicleData.battery_cell_HIGH_temp;
  msg.data[3] = (uint8_t)vehicleData.battery_cell_LOW_temp;
  msg.data[4] = (uint8_t)vehicleData.battery_cell_AVG_temp;
  msg.data[5] = vehicleData.battery_cell_ID_HIGH_temp;
  msg.data[6] = vehicleData.battery_cell_ID_LOW_temp;
  CAN.write(msg);
  
  // CAN ID 0x602: Battery Current/Voltage and Cell Voltages
  // Bytes 0-1: Battery Current (signed 16-bit / 10)
  // Bytes 2-3: Battery Voltage (uint16 / 10)
  // Bytes 4-5: Low Cell Voltage ((byte4 * byte5)/10000)
  // Bytes 6-7: High Cell Voltage ((byte6 * byte7)/10000)
  msg.id = CAN_ID::BATTERY_VOLT_CURRENT;
  msg.data_length = 8;
  memset(msg.data, 0, 8);
  int16_t battery_current_scaled = (int16_t)(vehicleData.battery_current * 10.0f);
  msg.data[0] = battery_current_scaled & 0xFF;
  msg.data[1] = (battery_current_scaled >> 8) & 0xFF;
  uint16_t battery_volt_scaled = (uint16_t)(vehicleData.battery_volt * 10.0f);
  msg.data[2] = battery_volt_scaled & 0xFF;
  msg.data[3] = (battery_volt_scaled >> 8) & 0xFF;
  // Low cell voltage: store as two bytes that when multiplied give the value * 10000
  // For simplicity, we'll store the value * 10000 as uint16
  uint16_t low_cell_volt_scaled = (uint16_t)(vehicleData.battery_cell_LOW_volt * 10000.0f);
  msg.data[4] = low_cell_volt_scaled & 0xFF;
  msg.data[5] = (low_cell_volt_scaled >> 8) & 0xFF;
  uint16_t high_cell_volt_scaled = (uint16_t)(vehicleData.battery_cell_HIGH_volt * 10000.0f);
  msg.data[6] = high_cell_volt_scaled & 0xFF;
  msg.data[7] = (high_cell_volt_scaled >> 8) & 0xFF;
  CAN.write(msg);
  
  // CAN ID 0x603: Avg Cell Voltage (bytes 0-1: uint16 / 10000)
  uint16_t avg_cell_volt_scaled = (uint16_t)(vehicleData.battery_cell_AVG_volt * 10000.0f);
  sendUint16Scaled(CAN_ID::BATTERY_AVG_CELL_VOLT, avg_cell_volt_scaled, 0);
}

void sendMPPTData() {
  // CAN ID 0x200: MPPT1 Wattage (bytes 4-7: float = voltOut*currentOut)
  sendFloatAtOffset(CAN_ID::MPPT1_WATTAGE, vehicleData.MPPT1_power, 4);
  
  // CAN ID 0x210: MPPT2 Wattage (bytes 4-7: float = voltOut*currentOut)
  sendFloatAtOffset(CAN_ID::MPPT2_WATTAGE, vehicleData.MPPT2_power, 4);
  
  // CAN ID 0x220: MPPT3 Wattage (bytes 4-7: float = voltOut*currentOut)
  sendFloatAtOffset(CAN_ID::MPPT3_WATTAGE, vehicleData.MPPT3_power, 4);
}

