#include "LoRa_functions.h"
#include <stdlib.h>
#include "Serial_commands.h"
#include "CAN_functions.h"

const unsigned long ACK_TIMEOUT_MS = 6000;
const unsigned long ACK_POLL_DELAY_MS = 25;

uint32_t packetCounter = 0;
uint16_t configSequence = 0;

// LoRa data variable to store CAN messages
String pendingPayload = "";
bool hasPendingPayload = false;

// Duty cycle management for 10% duty cycle (869.400-869.650 MHz sub-band)
unsigned long lastTransmissionStartTime = 0;
unsigned long lastTransmissionDuration = 0;
unsigned long dutyCycleWaitUntil = 0;

// Test configuration
constexpr int PACKETS_PER_CONFIG = 100;
constexpr int DEFAULT_SF = 7;
constexpr long DEFAULT_BW = 62500;
constexpr int DEFAULT_TP = 10;

constexpr int TEST_SPREADING_FACTORS[] = {7, 8, 9, 10, 11, 12};
constexpr long TEST_BANDWIDTHS[] = {62500, 125000, 250000, 500000};
constexpr int TEST_TX_POWERS[] = {2, 12, 22};  // Low, Medium (normal), High TX power levels (max 22 dBm with +3 dBi antenna = 25 dBm EIRP, safe below 26 dBm limit)
constexpr size_t TEST_SF_COUNT = sizeof(TEST_SPREADING_FACTORS) / sizeof(TEST_SPREADING_FACTORS[0]);
constexpr size_t TEST_BW_COUNT = sizeof(TEST_BANDWIDTHS) / sizeof(TEST_BANDWIDTHS[0]);
constexpr size_t TEST_TP_COUNT = sizeof(TEST_TX_POWERS) / sizeof(TEST_TX_POWERS[0]);

// Test state
struct TestState {
  bool running = false;
  size_t sfIndex = 0;          // Current SF index (0-5)
  size_t bwIndex = 0;          // Current BW index (0-3)
  size_t tpIndex = 0;          // Current TX Power index (0-2)
  unsigned long packetsSent = 0;  // Packets sent for current config
  unsigned long totalPacketsSent = 0;  // Total packets sent across all tests
  unsigned long testStartTime = 0;
  unsigned long configStartTime = 0;
};

TestState testState;

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

String toHex4(uint16_t value) {
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "%04X", value);
  return String(buffer);
}

String buildCfgBasePayload(int sfValue, long sbwValue, int tpValue, uint16_t seqValue) {
  return "CFG sf=" + String(sfValue) + " sbw=" + String(sbwValue) + " tp=" + String(tpValue) + " seq=" + String(seqValue);
}

String appendCrcToPayload(const String &basePayload) {
  uint16_t crc = crc16Ccitt(reinterpret_cast<const uint8_t *>(basePayload.c_str()), basePayload.length());
  return basePayload + " crc=" + toHex4(crc);
}

bool parseAckPayload(const String &response, int &sfValue, long &sbwValue, int &tpValue, uint16_t &seqValue, uint16_t &crcValue) {
  String trimmed = response;
  trimmed.trim();
  if (trimmed.length() < 3) return false;

  String prefix = trimmed.substring(0, 3);
  prefix.toUpperCase();
  if (prefix != "ACK") return false;

  String payload = trimmed.substring(3);
  payload.trim();
  if (payload.length() == 0) return false;

  bool hasSF = false;
  bool hasSBW = false;
  bool hasTP = false;
  bool hasSeq = false;
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
        sfValue = lower.substring(3).toInt();
        hasSF = true;
      } else if (lower.startsWith("sbw=")) {
        sbwValue = lower.substring(4).toInt();
        hasSBW = true;
      } else if (lower.startsWith("tp=")) {
        tpValue = lower.substring(3).toInt();
        hasTP = true;
      } else if (lower.startsWith("seq=")) {
        seqValue = static_cast<uint16_t>(lower.substring(4).toInt());
        hasSeq = true;
      } else if (lower.startsWith("crc=")) {
        crcValue = static_cast<uint16_t>(strtoul(lower.substring(4).c_str(), nullptr, 16));
        hasCrc = true;
      }
    }

    start = end + 1;
  }

  return hasSF && hasSBW && hasTP && hasSeq && hasCrc;
}

bool ackMatchesAll(const String &response, int sfValue, long sbwValue, int tpValue, uint16_t seqValue) {
  int ackSf = -1;
  long ackSbw = -1;
  int ackTp = -1;
  uint16_t ackSeq = 0;
  uint16_t ackCrc = 0;
  if (!parseAckPayload(response, ackSf, ackSbw, ackTp, ackSeq, ackCrc)) {
    return false;
  }

  if (ackSf != sfValue || ackSbw != sbwValue || ackTp != tpValue || ackSeq != seqValue) {
    return false;
  }

  String basePayload = "ACK sf=" + String(ackSf) + " sbw=" + String(ackSbw) + " tp=" + String(ackTp) + " seq=" + String(ackSeq);
  uint16_t expectedCrc = crc16Ccitt(reinterpret_cast<const uint8_t *>(basePayload.c_str()), basePayload.length());
  return expectedCrc == ackCrc;
}

bool waitForAckAll(int sfValue, long sbwValue, int tpValue, uint16_t seqValue) {
  unsigned long start = millis();
  LoRa.receive();
  while (millis() - start < ACK_TIMEOUT_MS) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      String response = LoRa.readString();
      response.trim();
      if (ackMatchesAll(response, sfValue, sbwValue, tpValue, seqValue)) {
        Serial.print("Received ACK: ");
        Serial.println(response);
        return true;
      }
      Serial.print("Ignoring unexpected response while waiting for ACK: ");
      Serial.println(response);
    }
    delay(ACK_POLL_DELAY_MS);
  }

  Serial.println("ACK wait timed out.");
  return false;
}

bool sendLoRaMessage(const String &payload) {
  // Duty cycle management: wait if we need to maintain 10% duty cycle
  unsigned long now = millis();
  if (now < dutyCycleWaitUntil) {
    // We need to wait to maintain 10% duty cycle
    unsigned long remainingWait = dutyCycleWaitUntil - now;
    delay(remainingWait);
    now = millis();
  }

  // Start transmission and record start time
  lastTransmissionStartTime = millis();
  
  if (LoRa.beginPacket() == 0) {
    Serial.println("Failed to start LoRa packet.");
    return false;
  }

  LoRa.print(payload);
  
  // endPacket() is blocking and waits for transmission to complete
  int result = LoRa.endPacket();
  if (result == 1) {
    // Calculate transmission duration (endPacket() blocks until transmission completes)
    unsigned long transmissionEndTime = millis();
    lastTransmissionDuration = transmissionEndTime - lastTransmissionStartTime;
    
    // Calculate when we can transmit again (wait 9x transmission time for 10% duty cycle)
    // This ensures 10% duty cycle: 1 unit transmit, 9 units wait = 10% on time
    dutyCycleWaitUntil = transmissionEndTime + (lastTransmissionDuration * 9);
    
    Serial.println("LoRa packet sent.");
    return true;
  } else {
    Serial.printf("LoRa packet send failed (code %d).\n", result);
    return false;
  }
}

bool notifyParameterChange(const String &parameter, long value) {
  int targetSf = sf;
  long targetSbw = sbw;
  int targetTp = txPower;
  String lowerParam = parameter;
  lowerParam.toLowerCase();
  if (lowerParam == "sf") {
    targetSf = static_cast<int>(value);
  } else if (lowerParam == "sbw") {
    targetSbw = value;
  } else if (lowerParam == "tp") {
    targetTp = static_cast<int>(value);
  }

  uint16_t seqValue = configSequence++;
  String basePayload = buildCfgBasePayload(targetSf, targetSbw, targetTp, seqValue);
  String payload = appendCrcToPayload(basePayload);
  Serial.print("Broadcasting parameter change: ");
  Serial.println(payload);

  if (!sendLoRaMessage(payload)) {
    Serial.println("Unable to send parameter change.");
    return false;
  }

  if (waitForAckAll(targetSf, targetSbw, targetTp, seqValue)) {
    Serial.println("ACK received, proceeding with local update.");
    return true;
  }

  Serial.println("No ACK received, aborting local parameter change.");
  return false;
}

bool notifyAllParametersChange(int sfValue, long sbwValue, int tpValue) {
  uint16_t seqValue = configSequence++;
  String basePayload = buildCfgBasePayload(sfValue, sbwValue, tpValue, seqValue);
  String payload = appendCrcToPayload(basePayload);
  unsigned long attemptCount = 0;
  unsigned long lastStatusTime = 0;
  const unsigned long STATUS_INTERVAL_MS = 5000;  // Print status every 5 seconds
  
  // Keep retrying until we get an ACK
  while (true) {
    attemptCount++;
    
    Serial.print("Broadcasting all parameter changes (attempt ");
    Serial.print(attemptCount);
    Serial.print("): ");
    Serial.println(payload);

    if (!sendLoRaMessage(payload)) {
      Serial.println("Unable to send parameter changes, retrying...");
      delay(1000);  // Wait 1 second before retry
      continue;
    }

    if (waitForAckAll(sfValue, sbwValue, tpValue, seqValue)) {
      Serial.println("ACK received for all parameters, proceeding with local update.");
      return true;
    }

    // No ACK received, print status periodically
    unsigned long now = millis();
    if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
      Serial.println("[TEST] No ACK received yet, retrying... (receiver may be out of range or not responding)");
      lastStatusTime = now;
    }
    
    // Small delay before retry
    delay(500);
  }
}

// Check if a configuration is valid (not exceeding max tx interval)
// SF 10 with BW 62500 = 9890 ms (baseline, max acceptable)
// Increasing SF by 1 doubles tx interval, increasing BW by 1 step halves it
bool isConfigurationValid(int sf, long bw) {
  // Calculate relative tx interval compared to baseline (SF 10, BW 62500 = 1.0)
  // Each SF step above 10 doubles the interval
  // Each BW step above 62500 halves the interval
  float txIntervalMultiplier = 1.0f;
  
  // Adjust for SF (SF 10 is baseline)
  if (sf > 10) {
    // SF 11 = 2x, SF 12 = 4x
    txIntervalMultiplier *= (1 << (sf - 10));  // 2^(sf-10)
  } else if (sf < 10) {
    // SF 9 = 0.5x, SF 8 = 0.25x, SF 7 = 0.125x
    txIntervalMultiplier /= (1 << (10 - sf));  // 1/(2^(10-sf))
  }
  
  // Adjust for BW (62500 is baseline)
  // BW 62500 = 1.0x, 125000 = 0.5x, 250000 = 0.25x, 500000 = 0.125x
  if (bw == 125000) {
    txIntervalMultiplier *= 0.5f;
  } else if (bw == 250000) {
    txIntervalMultiplier *= 0.25f;
  } else if (bw == 500000) {
    txIntervalMultiplier *= 0.125f;
  }
  // bw == 62500 stays at 1.0x
  
  // Check if resulting tx interval would exceed 9890 ms (baseline)
  // txInterval = 9890 * txIntervalMultiplier
  // We want txInterval <= 9890, so txIntervalMultiplier <= 1.0
  return txIntervalMultiplier <= 1.0f;
}

bool applyTestConfiguration() {
  int currentSF = TEST_SPREADING_FACTORS[testState.sfIndex];
  long currentBW = TEST_BANDWIDTHS[testState.bwIndex];
  int currentTP = TEST_TX_POWERS[testState.tpIndex];
  
  // Double-check that configuration is valid (should already be checked by advanceTestConfiguration)
  if (!isConfigurationValid(currentSF, currentBW)) {
    Serial.printf("\n[TEST] WARNING: Configuration SF: %d, BW: %ld Hz exceeds max tx interval, skipping...\n",
                  currentSF, currentBW);
    // Try to advance to next valid configuration
    if (advanceTestConfiguration()) {
      // Recursively apply the next valid configuration
      return applyTestConfiguration();
    } else {
      // No more valid configurations
      return false;
    }
  }
  
  size_t configNum = testState.sfIndex * TEST_BW_COUNT * TEST_TP_COUNT + 
                     testState.bwIndex * TEST_TP_COUNT + 
                     testState.tpIndex + 1;
  size_t totalConfigs = TEST_SF_COUNT * TEST_BW_COUNT * TEST_TP_COUNT;
  
  Serial.printf("\n[TEST] SF: %d | BW: %ld Hz | TP: %d dBm | Config %zu/%zu\n",
                currentSF,
                currentBW,
                currentTP,
                configNum,
                totalConfigs);
  
  // Send all parameters in one CFG message to receiver and wait for ACK
  // notifyAllParametersChange() will retry indefinitely until ACK is received
  Serial.println("[TEST] Sending all parameters (SF, BW, TP) to receiver...");
  notifyAllParametersChange(currentSF, currentBW, currentTP);
  // If we reach here, ACK was received successfully
  
  // Only apply configuration locally AFTER receiving ACK for all parameters
  sf = currentSF;
  sbw = currentBW;
  txPower = currentTP;
  updateFlag = 1;
  applyLoRaConfig();
  
  testState.configStartTime = millis();
  testState.packetsSent = 0;
  
  Serial.println("[TEST] Configuration applied and synchronized with receiver. Starting packet transmission...");
  return true;
}

bool advanceTestConfiguration() {
  // Advance to next TX Power
  if (testState.tpIndex + 1 < TEST_TP_COUNT) {
    testState.tpIndex++;
    // Check if this configuration is valid, if not skip it
    int currentSF = TEST_SPREADING_FACTORS[testState.sfIndex];
    long currentBW = TEST_BANDWIDTHS[testState.bwIndex];
    if (isConfigurationValid(currentSF, currentBW)) {
      return true;
    } else {
      // Skip this invalid configuration and continue advancing
      return advanceTestConfiguration();
    }
  }
  
  // Advance to next bandwidth
  if (testState.bwIndex + 1 < TEST_BW_COUNT) {
    testState.bwIndex++;
    testState.tpIndex = 0;
    // Check if this configuration is valid, if not skip it
    int currentSF = TEST_SPREADING_FACTORS[testState.sfIndex];
    long currentBW = TEST_BANDWIDTHS[testState.bwIndex];
    if (isConfigurationValid(currentSF, currentBW)) {
      return true;
    } else {
      // Skip this invalid configuration and continue advancing
      return advanceTestConfiguration();
    }
  }
  
  // Advance to next spreading factor
  if (testState.sfIndex + 1 < TEST_SF_COUNT) {
    testState.sfIndex++;
    testState.bwIndex = 0;
    testState.tpIndex = 0;
    // Check if this configuration is valid, if not skip it
    int currentSF = TEST_SPREADING_FACTORS[testState.sfIndex];
    long currentBW = TEST_BANDWIDTHS[testState.bwIndex];
    if (isConfigurationValid(currentSF, currentBW)) {
      return true;
    } else {
      // Skip this invalid configuration and continue advancing
      return advanceTestConfiguration();
    }
  }
  
    // All configs done
  return false;
}

void finalizeCurrentConfig() {
  unsigned long configDuration = millis() - testState.configStartTime;
  float avgInterval = testState.packetsSent > 0 ? (float)configDuration / testState.packetsSent : 0.0f;
  
  Serial.printf("[TEST] Config completed: %lu packets in %lu ms (avg: %.1f ms/packet)\n",
                testState.packetsSent,
                configDuration,
                avgInterval);
}

void sendDefaultsToReceiver() {
  Serial.println("[TEST] Reverting receiver to default settings...");
  notifyAllParametersChange(DEFAULT_SF, DEFAULT_BW, DEFAULT_TP);
  sf = DEFAULT_SF;
  sbw = DEFAULT_BW;
  txPower = DEFAULT_TP;
  updateFlag = 1;
  applyLoRaConfig();
  Serial.println("[TEST] Receiver defaults acknowledged.");
}

void startTest() {
  if (testState.running) {
    Serial.println("[TEST] Test already running. Use 'stoptest' to stop first.");
    return;
  }
  
  Serial.println("\n[TEST] ========================================");
  Serial.println("[TEST] Starting test sequence");
  Serial.printf("[TEST] Configs: %zu (SF: %zu, BW: %zu, TP: %zu)\n", 
               TEST_SF_COUNT * TEST_BW_COUNT * TEST_TP_COUNT, 
               TEST_SF_COUNT, 
               TEST_BW_COUNT,
               TEST_TP_COUNT);
  Serial.println("[TEST] Note: Configurations with tx interval > 9890 ms will be skipped");
  Serial.printf("[TEST] Packets per config: %d\n", PACKETS_PER_CONFIG);
  Serial.println("[TEST] ========================================\n");
  
  // Initialize test state
  testState.running = true;
  testState.sfIndex = 0;
  testState.bwIndex = 0;
  testState.tpIndex = 0;
  testState.packetsSent = 0;
  testState.totalPacketsSent = 0;
  testState.testStartTime = millis();
  
  // Ensure initial configuration is valid (SF 7, BW 62500 should be valid)
  int initialSF = TEST_SPREADING_FACTORS[testState.sfIndex];
  long initialBW = TEST_BANDWIDTHS[testState.bwIndex];
  if (!isConfigurationValid(initialSF, initialBW)) {
    Serial.println("[TEST] WARNING: Initial configuration is invalid, skipping to first valid config...");
    if (!advanceTestConfiguration()) {
      Serial.println("[TEST] ERROR: No valid configurations found!");
      testState.running = false;
      return;
    }
  }
  
  Serial.println("[TEST] Synchronizing initial parameters with receiver...");
  
  // Apply and send initial configuration to receiver
  // Will retry until ACK is received (no pause on failure)
  if (!applyTestConfiguration()) {
    return;  // No valid configurations found
  }
}

void stopTest() {
  if (!testState.running) {
    Serial.println("[TEST] No test is currently running.");
    return;
  }
  
  unsigned long totalDuration = millis() - testState.testStartTime;
  
  Serial.println("\n[TEST] ========================================");
  Serial.println("[TEST] Test stopped");
  Serial.printf("[TEST] Total packets sent: %lu\n", testState.totalPacketsSent);
  Serial.printf("[TEST] Total duration: %lu ms (%.1f minutes)\n", totalDuration, totalDuration / 60000.0f);
  Serial.println("[TEST] ========================================\n");
  
  testState.running = false;
}

void skipConfig() {
  if (!testState.running) {
    Serial.println("[TEST] No test is currently running.");
    return;
  }
  
  // Finalize current config if it has sent any packets
  if (testState.packetsSent > 0) {
    Serial.println("[TEST] Skipping current configuration...");
    finalizeCurrentConfig();
  } else {
    Serial.println("[TEST] Skipping current configuration (no packets sent yet)...");
  }
  
  // Advance to next configuration (will automatically skip invalid ones)
  if (advanceTestConfiguration()) {
    Serial.println("[TEST] Advancing to next configuration...");
    // Try to apply new configuration (will retry until ACK is received)
    // applyTestConfiguration() will also skip invalid configs if needed
    if (!applyTestConfiguration()) {
      return;  // No more valid configurations available
    }
  } else {
    // All configs done
    Serial.println("\n[TEST] ========================================");
    Serial.println("[TEST] All configurations completed.");
    Serial.printf("[TEST] Total packets sent: %lu\n", testState.totalPacketsSent);
    Serial.println("[TEST] ========================================\n");
    stopTest();
    sendDefaultsToReceiver();
  }
}

void printTestStatus() {
  if (!testState.running) {
    Serial.println("[TEST] No test is currently running.");
    return;
  }
  
  Serial.println("\n[TEST] ========================================");
  Serial.printf("[TEST] Status: %s\n", testState.running ? "RUNNING" : "STOPPED");
  size_t configNum = testState.sfIndex * TEST_BW_COUNT * TEST_TP_COUNT + 
                     testState.bwIndex * TEST_TP_COUNT + 
                     testState.tpIndex + 1;
  size_t totalConfigs = TEST_SF_COUNT * TEST_BW_COUNT * TEST_TP_COUNT;
  Serial.printf("[TEST] Current config: SF=%d, BW=%ld Hz, TP=%d dBm (%zu/%zu)\n",
                TEST_SPREADING_FACTORS[testState.sfIndex],
                TEST_BANDWIDTHS[testState.bwIndex],
                TEST_TX_POWERS[testState.tpIndex],
                configNum,
                totalConfigs);
  Serial.printf("[TEST] Packets this config: %lu/%d\n", testState.packetsSent, PACKETS_PER_CONFIG);
  Serial.printf("[TEST] Total packets sent: %lu\n", testState.totalPacketsSent);
  unsigned long elapsed = millis() - testState.testStartTime;
  Serial.printf("[TEST] Test duration: %lu ms (%.1f minutes)\n", elapsed, elapsed / 60000.0f);
  Serial.println("[TEST] ========================================\n");
}

void setup() {
  serialSetup();
  setupLoRa();
  setupCAN();
  Serial.println("LoRa Test Sender Ready");
  Serial.println("CAN bus initialized - ready for test mode");
  Serial.println("Use 'starttest' to begin test sequence");
  Serial.println("Transmission rate controlled by 10% duty cycle limit");
}

void loop() {
  // Handle serial commands
  handleSerialCommands(nullptr, notifyParameterChange);

  // Read CAN messages from Arduino (only used during test)
  CanFrame canFrame;
  while (readCANFrame(canFrame)) {
    pendingPayload = formatCanFrame(canFrame);
    hasPendingPayload = true;
  }

  // Handle test state
  if (testState.running) {
    // Check if current config is complete
    if (testState.packetsSent >= PACKETS_PER_CONFIG) {
      finalizeCurrentConfig();
      
      // Advance to next configuration
      if (advanceTestConfiguration()) {
        // Try to apply new configuration (will retry until ACK is received)
        if (!applyTestConfiguration()) {
          return;  // No more valid configurations available
        }
      } else {
        // All configs done
        Serial.println("\n[TEST] ========================================");
        Serial.println("[TEST] All configurations completed.");
        Serial.printf("[TEST] Total packets sent: %lu\n", testState.totalPacketsSent);
        Serial.println("[TEST] ========================================\n");
        stopTest();
        sendDefaultsToReceiver();
        return;
      }
    }
    
    // Send CAN data during test
    // Duty cycle management is handled inside sendLoRaMessage()
    if (hasPendingPayload && pendingPayload.length() > 0) {
      if (sendLoRaMessage(pendingPayload)) {
        testState.packetsSent++;
        testState.totalPacketsSent++;
        packetCounter++;
        
        // Progress indicator every 10 packets
        if (testState.packetsSent % 10 == 0) {
          Serial.printf("[TEST] Progress: %lu/%d packets (%.1f%%)\n",
                        testState.packetsSent,
                        PACKETS_PER_CONFIG,
                        100.0f * testState.packetsSent / PACKETS_PER_CONFIG);
        }
      } else {
        Serial.println("[TEST] Failed to send CAN data.");
      }
      
      // Clear CAN data after sending (or failed attempt)
      pendingPayload = "";
      hasPendingPayload = false;
    }
  }
  // If test is not running, just read CAN messages but don't send them
}

