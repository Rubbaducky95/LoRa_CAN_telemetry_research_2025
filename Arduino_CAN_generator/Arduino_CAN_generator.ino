#include <Arduino.h>
#include <math.h>
#include "CAN_functions.h"

// CAN transmission interval (milliseconds)
const unsigned long CAN_SEND_INTERVAL = 100;  // 100ms = 10Hz

// Timing
unsigned long lastSendTime = 0;

// Simulated data values (you can modify these or connect real sensors)
struct VehicleData {
  // Motor data
  float motor_current = 0.0;
  float motor_temp = 25.0;
  
  // Vehicle data
  float velocity = 0.0;
  float distance_travelled = 0.0;
  
  // Battery data
  float battery_volt = 150.0;
  float battery_current = 0.0;
  float battery_cell_LOW_volt = 3.6;
  float battery_cell_HIGH_volt = 3.7;
  float battery_cell_AVG_volt = 3.65;
  float battery_cell_LOW_temp = 20.0;
  float battery_cell_HIGH_temp = 25.0;
  float battery_cell_AVG_temp = 22.5;
  uint8_t battery_cell_ID_HIGH_temp = 1;
  uint8_t battery_cell_ID_LOW_temp = 2;
  float BMS_temp = 23.0;
  
  // MPPT1 data
  float MPPT1_input_voltage = 75.0;
  float MPPT1_input_current = 0.0;
  float MPPT1_output_voltage = 151.0;
  float MPPT1_output_current = 0.0;
  float MPPT1_power = 0.0;
  
  // MPPT2 data
  float MPPT2_input_voltage = 75.0;
  float MPPT2_input_current = 0.0;
  float MPPT2_output_voltage = 151.0;
  float MPPT2_output_current = 0.0;
  float MPPT2_power = 0.0;
  
  // MPPT3 data
  float MPPT3_input_voltage = 75.0;
  float MPPT3_input_current = 0.0;
  float MPPT3_output_voltage = 151.0;
  float MPPT3_output_current = 0.0;
  float MPPT3_power = 0.0;
  
  float MPPT_total_watt = 0.0;
};

VehicleData vehicleData;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  
  Serial.println("Arduino R4 Minima CAN Data Generator");
  Serial.println("====================================");
  Serial.println("CAN Pin Configuration (FIXED):");
  Serial.println("  TX: D4 (CANTX0) -> Connect to CAN transceiver TXD");
  Serial.println("  RX: D5 (CANRX0) -> Connect to CAN transceiver RXD");
  Serial.println();
  
  if (setupCAN()) {
    Serial.println("CAN bus initialized successfully");
  } else {
    Serial.println("ERROR: CAN bus initialization failed!");
    Serial.println("Check your CAN transceiver connections:");
    Serial.println("  - D4 (TX) connected to transceiver TXD");
    Serial.println("  - D5 (RX) connected to transceiver RXD");
    Serial.println("  - Transceiver powered (5V and GND)");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("Starting CAN data transmission...");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update simulated data (you can replace this with real sensor readings)
  updateSimulatedData();
  
  // Send CAN messages at regular intervals
  if (currentTime - lastSendTime >= CAN_SEND_INTERVAL) {
    sendAllCANMessages();
    lastSendTime = currentTime;
  }
  
  delay(10);
}

void updateSimulatedData() {
  // Simulate some variation in the data
  static unsigned long counter = 0;
  counter++;
  
  // Simulate motor current (0-10A range)
  vehicleData.motor_current = 5.0 + 3.0 * sin(counter * 0.01);
  
  // Simulate velocity (0-50 km/h range)
  vehicleData.velocity = 20.0 + 15.0 * sin(counter * 0.005);
  
  // Increment distance
  vehicleData.distance_travelled += vehicleData.velocity * (CAN_SEND_INTERVAL / 3600000.0);
  
  // Simulate battery voltage (140-155V range)
  vehicleData.battery_volt = 150.0 + 5.0 * sin(counter * 0.002);
  
  // Simulate battery current
  vehicleData.battery_current = vehicleData.motor_current * 0.8;
  
  // Simulate cell voltages
  vehicleData.battery_cell_LOW_volt = 3.6 + 0.1 * sin(counter * 0.003);
  vehicleData.battery_cell_HIGH_volt = 3.7 + 0.1 * sin(counter * 0.003);
  vehicleData.battery_cell_AVG_volt = (vehicleData.battery_cell_LOW_volt + vehicleData.battery_cell_HIGH_volt) / 2.0;
  
  // Simulate temperatures
  vehicleData.motor_temp = 25.0 + 5.0 * sin(counter * 0.001);
  vehicleData.BMS_temp = 23.0 + 3.0 * sin(counter * 0.001);
  vehicleData.battery_cell_LOW_temp = 20.0 + 3.0 * sin(counter * 0.001);
  vehicleData.battery_cell_HIGH_temp = 25.0 + 3.0 * sin(counter * 0.001);
  vehicleData.battery_cell_AVG_temp = (vehicleData.battery_cell_LOW_temp + vehicleData.battery_cell_HIGH_temp) / 2.0;
  
  // Simulate MPPT data
  vehicleData.MPPT1_input_voltage = 75.0 + 2.0 * sin(counter * 0.002);
  vehicleData.MPPT1_input_current = 0.5 + 0.3 * sin(counter * 0.01);
  vehicleData.MPPT1_output_voltage = 151.0 + 1.0 * sin(counter * 0.002);
  vehicleData.MPPT1_output_current = vehicleData.MPPT1_input_current * 0.95;
  vehicleData.MPPT1_power = vehicleData.MPPT1_output_voltage * vehicleData.MPPT1_output_current;
  
  vehicleData.MPPT2_input_voltage = 75.0 + 1.5 * sin(counter * 0.002 + 1.0);
  vehicleData.MPPT2_input_current = 0.4 + 0.2 * sin(counter * 0.01 + 1.0);
  vehicleData.MPPT2_output_voltage = 151.0 + 0.8 * sin(counter * 0.002 + 1.0);
  vehicleData.MPPT2_output_current = vehicleData.MPPT2_input_current * 0.95;
  vehicleData.MPPT2_power = vehicleData.MPPT2_output_voltage * vehicleData.MPPT2_output_current;
  
  vehicleData.MPPT3_input_voltage = 75.0 + 1.8 * sin(counter * 0.002 + 2.0);
  vehicleData.MPPT3_input_current = 0.45 + 0.25 * sin(counter * 0.01 + 2.0);
  vehicleData.MPPT3_output_voltage = 151.0 + 0.9 * sin(counter * 0.002 + 2.0);
  vehicleData.MPPT3_output_current = vehicleData.MPPT3_input_current * 0.95;
  vehicleData.MPPT3_power = vehicleData.MPPT3_output_voltage * vehicleData.MPPT3_output_current;
  
  vehicleData.MPPT_total_watt = vehicleData.MPPT1_power + vehicleData.MPPT2_power + vehicleData.MPPT3_power;
}

void sendAllCANMessages() {
  // Send motor data
  sendMotorData();
  
  // Send vehicle data
  sendVehicleData();
  
  // Send battery data
  sendBatteryData();
  
  // Send MPPT data
  sendMPPTData();
}

