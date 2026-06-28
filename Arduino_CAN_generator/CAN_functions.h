#ifndef CAN_FUNCTIONS_H
#define CAN_FUNCTIONS_H

#include <Arduino.h>

// Arduino R4 Minima CAN pins (fixed, cannot be changed)
// D4 = CANTX0 (CAN Transmit) - Connect to CAN transceiver TXD pin
// D5 = CANRX0 (CAN Receive)  - Connect to CAN transceiver RXD pin

// CAN message IDs
// Based on CAN signal map table
namespace CAN_ID {
  // Motor data
  constexpr uint16_t MOTOR_CURRENT = 0x402;        // bytes 4-7: float
  constexpr uint16_t MOTOR_MC_TEMP = 0x40B;        // bytes 0-3: Motor Temp (float), bytes 4-7: MC Temp (float)
  
  // Vehicle data
  constexpr uint16_t VELOCITY = 0x403;             // bytes 4-7: float
  constexpr uint16_t DISTANCE_TRAVELLED = 0x40E;   // bytes 0-3: float
  
  // Battery data
  constexpr uint16_t BATTERY_TEMPS = 0x601;        // bytes 1-6: BMS temp, High/Low/Avg cell temps, High/Low temp cell IDs (uint8)
  constexpr uint16_t BATTERY_VOLT_CURRENT = 0x602; // bytes 0-7: Battery current/voltage, Low/High cell voltages
  constexpr uint16_t BATTERY_AVG_CELL_VOLT = 0x603; // bytes 0-1: Avg cell voltage (uint16 / 10000)
  
  // MPPT data
  constexpr uint16_t MPPT1_WATTAGE = 0x200;        // bytes 4-7: float (voltOut*currentOut)
  constexpr uint16_t MPPT2_WATTAGE = 0x210;        // bytes 4-7: float (voltOut*currentOut)
  constexpr uint16_t MPPT3_WATTAGE = 0x220;        // bytes 4-7: float (voltOut*currentOut)
}

// Function declarations
bool setupCAN();
void sendMotorData();
void sendVehicleData();
void sendBatteryData();
void sendMPPTData();

// Helper functions
void sendFloat(uint16_t canId, float value1, float value2);
void sendFloatAtOffset(uint16_t canId, float value, uint8_t byteOffset);
void sendUint8(uint16_t canId, uint8_t value, uint8_t byteOffset);
void sendInt16Scaled(uint16_t canId, int16_t value, uint8_t byteOffset);
void sendUint16Scaled(uint16_t canId, uint16_t value, uint8_t byteOffset);

#endif

