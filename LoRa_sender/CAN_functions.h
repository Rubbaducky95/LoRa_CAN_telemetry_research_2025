#ifndef CAN_FUNCTIONS_H
#define CAN_FUNCTIONS_H

#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// CAN pins for ESP32S3 (must not conflict with LoRa pins: 2, 5, 14)
constexpr int CAN_TX_PIN = 16;
constexpr int CAN_RX_PIN = 4;

extern CanFrame msg;

void setupCAN();
String formatCanFrame(const CanFrame &frame);
bool readCANFrame(CanFrame &frame);

#endif

