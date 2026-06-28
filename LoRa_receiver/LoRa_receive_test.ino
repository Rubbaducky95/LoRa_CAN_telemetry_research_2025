#include "LoRa_functions.h"
#include "Serial_commands.h"
#include <stdlib.h>

// SMA filter variables
const int SMA_WINDOW_SIZE = 5; // Change if bigger window is needed
float smaBuffer[SMA_WINDOW_SIZE];
int smaIndex = 0;
int smaCount = 0;
float smaSum = 0.0;
float smaRSSI = 0.0;

// Kalman filter variables
float processNoise = 0.1;     // Q
float measurementNoise = 9.0; // R (set to variance , e.g. std^2)
float estimatedError = 5.0;    // P (initial estimate error)
float estimatedRSSI = 0.0;
float kalmanRSSI = 0.0;    // Will be set after first iteration
bool haveEstimate = false;

float packetRSSI;
String LoRaData;
uint16_t lastConfigSeq = 0;
bool hasLastConfigSeq = false;

float kalmanFilter(float measurement) {

  if(!haveEstimate) {
    estimatedRSSI = measurement;
    haveEstimate = true;
    return estimatedRSSI;
  }

  // Prediction step
  estimatedError += processNoise;
  //Update step
  float kalmanGain = estimatedError / (estimatedError + measurementNoise);
  // Update estimated value
  estimatedRSSI = estimatedRSSI + kalmanGain * (measurement - estimatedRSSI);
  //Update error covariance
  estimatedError = (1.0 - kalmanGain)*estimatedError;

  return estimatedRSSI;
}

void smaReset() {
  for (int i = 0; i < SMA_WINDOW_SIZE; ++i) smaBuffer[i] = 0.0;
  smaIndex = 0;
  smaCount = 0;
  smaSum = 0.0;
}

float simpleMovingAverage(float measurement) {

  if (smaCount < SMA_WINDOW_SIZE) {
    // Buffer not yet full
    smaBuffer[smaIndex] = measurement;
    smaSum += measurement;
    smaIndex = (smaIndex + 1) % SMA_WINDOW_SIZE;
    smaCount++;
    return smaSum / smaCount;        // average of what's available so far
  } else {
    // Buffer full
    smaSum -= smaBuffer[smaIndex];   // remove oldest
    smaBuffer[smaIndex] = measurement;  // overwrite it with the new value
    smaSum += measurement;             // add new
    smaIndex = (smaIndex + 1) % SMA_WINDOW_SIZE;
    return smaSum / SMA_WINDOW_SIZE; // constant-window average
  }
}

bool processRemoteParameterMessage(const String& message) {
  String trimmed = message;
  trimmed.trim();
  if (trimmed.length() == 0) return false;

  if (!trimmed.startsWith("CFG") && !trimmed.startsWith("cfg")) return false;

  String payload = trimmed.substring(3);
  payload.trim();
  if (payload.length() == 0) {
    Serial.println("Remote CFG message missing payload.");
    return false;
  }

  int newSfValue = -1;
  long newSbwValue = -1;
  int newTpValue = -1;
  int newSeqValue = -1;
  uint16_t newCrcValue = 0;
  bool hasCrc = false;

  int start = 0;
  while (start < payload.length()) {
    int end = payload.indexOf(' ', start);
    int comma = payload.indexOf(',', start);
    if (end == -1 || (comma != -1 && comma < end)) {
      end = comma;
    }
    if (end == -1) end = payload.length();

    String token = payload.substring(start, end);
    token.trim();
    if (token.length() > 0) {
      String lower = token;
      lower.toLowerCase();
      if (lower.startsWith("sf=")) {
        newSfValue = lower.substring(3).toInt();
      } else if (lower.startsWith("sbw=")) {
        newSbwValue = lower.substring(4).toInt();
      } else if (lower.startsWith("tp=")) {
        newTpValue = lower.substring(3).toInt();
      } else if (lower.startsWith("seq=")) {
        newSeqValue = lower.substring(4).toInt();
      } else if (lower.startsWith("crc=")) {
        newCrcValue = static_cast<uint16_t>(strtoul(lower.substring(4).c_str(), nullptr, 16));
        hasCrc = true;
      } else {
        Serial.println("Unknown CFG token: " + token);
      }
    }

    start = end + 1;
  }

  if (newSfValue == -1 || newSbwValue == -1 || newTpValue == -1 || newSeqValue == -1 || !hasCrc) {
    Serial.println("Remote CFG message missing required parameters (sf/sbw/tp/seq/crc).");
    return false;
  }

  bool valid = true;

  if (newSfValue != -1 && !isValidSpreadingFactor(newSfValue)) {
    Serial.printf("Remote SF invalid: %d\n", newSfValue);
    valid = false;
  }

  if (newSbwValue != -1 && !isValidBandwidth(newSbwValue)) {
    Serial.printf("Remote BW invalid: %ld\n", newSbwValue);
    valid = false;
  }

  if (newTpValue != -1 && !isValidTransmitPower(newTpValue)) {
    Serial.printf("Remote TP invalid: %d\n", newTpValue);
    valid = false;
  }

  if (!valid) return false;

  String basePayload = "CFG sf=" + String(newSfValue) + " sbw=" + String(newSbwValue) + " tp=" + String(newTpValue) + " seq=" + String(newSeqValue);
  uint16_t expectedCrc = crc16Ccitt(reinterpret_cast<const uint8_t *>(basePayload.c_str()), basePayload.length());
  if (expectedCrc != newCrcValue) {
    Serial.println("Remote CFG message CRC mismatch, ignoring.");
    return false;
  }

  int currentSf = sf;
  long currentSbw = sbw;
  int currentTp = txPower;

  int targetSf = (newSfValue != -1) ? newSfValue : currentSf;
  long targetSbw = (newSbwValue != -1) ? newSbwValue : currentSbw;
  int targetTp = (newTpValue != -1) ? newTpValue : currentTp;

  // Duplicate sequence with same active parameters: ACK only
  if (hasLastConfigSeq && newSeqValue == lastConfigSeq &&
      targetSf == currentSf && targetSbw == currentSbw && targetTp == currentTp) {
    if (!sendConfigAck(targetSf, targetSbw, targetTp, static_cast<uint16_t>(newSeqValue))) {
      Serial.println("Unable to transmit ACK for duplicate parameters.");
      return false;
    }
    Serial.println("Duplicate CFG sequence received. ACK sent to confirm synchronization.");
    return true;
  }

  // Always send ACK, even if parameters are already active
  // This ensures sender knows receiver is synchronized
  if (!sendConfigAck(targetSf, targetSbw, targetTp, static_cast<uint16_t>(newSeqValue))) {
    Serial.println("Unable to transmit ACK for parameters.");
    return false;
  }

  // Only update parameters if they actually changed
  if (targetSf == currentSf && targetSbw == currentSbw && targetTp == currentTp) {
    Serial.println("Remote parameters already active. ACK sent to confirm synchronization.");
    lastConfigSeq = static_cast<uint16_t>(newSeqValue);
    hasLastConfigSeq = true;
    return true;
  }

  // Parameters changed, update them
  sf = targetSf;
  sbw = targetSbw;
  txPower = targetTp;
  updateFlag = 1;
  applyLoRaConfig();
  // Ensure we're back in receive mode after applying config
  LoRa.receive();
  Serial.printf("Applied parameters from sender -> SF: %d, BW: %ld Hz, TP: %d dBm\n", sf, sbw, txPower);
  lastConfigSeq = static_cast<uint16_t>(newSeqValue);
  hasLastConfigSeq = true;
  return true;
}

void setup() {
  serialSetup();
  setupLoRa();
}

void loop() {

  handleSerialCommands();

  // Try to parse packet
  int packetSize = LoRa.parsePacket();

  if (packetSize) {    // received a packet
    //Serial.println("Message received!");
    while (LoRa.available()) {    // read packet
      LoRaData = LoRa.readString();
      processRemoteParameterMessage(LoRaData);
    }

    // RSSI of packet
    packetRSSI = LoRa.packetRssi();

    // Apply Kalman Filter
    kalmanRSSI = kalmanFilter(packetRSSI);

    // Apply Simple Moving Average
    smaRSSI = simpleMovingAverage(packetRSSI);
    
    // Print all data on one line for easier parsing
    Serial.printf("DATA|LoRa_data:%s|RSSI:%.4f|Kalman_RSSI:%.4f|SMA_RSSI:%.4f\n", 
                  LoRaData.c_str(), packetRSSI, kalmanRSSI, smaRSSI);
    
  }
}