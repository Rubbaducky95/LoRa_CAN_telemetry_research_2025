#include "CAN_functions.h"

CanFrame msg;

namespace {
constexpr uint32_t kDefaultCanSpeedKbps = 500;
}

void setupCAN() {
  ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
  ESP32Can.setRxQueueSize(10);
  ESP32Can.setTxQueueSize(10);

  const auto speed = ESP32Can.convertSpeed(kDefaultCanSpeedKbps);
  if (ESP32Can.begin(speed)) {
    Serial.printf("CAN bus started at %lu kbps\n", static_cast<unsigned long>(kDefaultCanSpeedKbps));
  } else {
    Serial.println("CAN bus failed to start!");
  }
}

bool readCANFrame(CanFrame &frame) {
  return ESP32Can.readFrame(frame, 0);
}

String formatCanFrame(const CanFrame &frame) {
  String payload;
  payload.reserve(64);

  payload += String(millis());
  payload += ",";

  String id = String(frame.identifier, HEX);
  id.toUpperCase();
  while (id.length() < 3) {
    id = "0" + id;
  }
  payload += id;
  payload += ",";

  payload += String(frame.data_length_code);

  for (int i = 0; i < frame.data_length_code; ++i) {
    payload += ",";
    uint8_t value = frame.data[i];
    if (value < 16) {
      payload += "0";
    }
    String byteValue = String(value, HEX);
    byteValue.toUpperCase();
    payload += byteValue;
  }

  return payload;
}

