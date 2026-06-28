#include "LoRa_functions.h"
#include <Arduino.h>

long sbw = 62500; // 20800, 62500, 125000, 250000 (default matches first test config)
int sf = 7;       // 6, 7, 8, 9, 10, 11, 12
int updateFlag = 1;
long newSbw = sbw;
int newSf = sf;
int txPower = 10;  // Safe default: 10 dBm (compliant with 3 dBi antenna)
int newTxPower = txPower;

namespace {

const long kValidBandwidths[] = {15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};
const size_t kValidBandwidthCount = sizeof(kValidBandwidths) / sizeof(kValidBandwidths[0]);
const int kMinTxPower = 2;
const int kMaxTxPower = 22;  // Max 22 dBm (with +3 dBi antenna = 25 dBm EIRP, safe below 26 dBm limit)
const int kAckRepeatCount = 3;
const unsigned long kAckRepeatDelayMs = 80;

}  // namespace

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

bool isValidBandwidth(long candidate) {
    for (size_t i = 0; i < kValidBandwidthCount; ++i) {
        if (candidate == kValidBandwidths[i]) return true;
    }
    return false;
}

bool isValidSpreadingFactor(int candidate) {
    return candidate >= 7 && candidate <= 12;
}

bool isValidTransmitPower(int candidate) {
    return candidate >= kMinTxPower && candidate <= kMaxTxPower;
}

bool queueLoRaParams(int newSfValue, long newSbwValue, int newTxPowerValue) {
    if (newSfValue != -1 && !isValidSpreadingFactor(newSfValue)) {
        Serial.printf("Rejected SF=%d (valid range 7-12)\n", newSfValue);
        return false;
    }

    if (newSbwValue != -1 && !isValidBandwidth(newSbwValue)) {
        Serial.printf("Rejected BW=%ld (unsupported)\n", newSbwValue);
        return false;
    }

    if (newTxPowerValue != -1 && !isValidTransmitPower(newTxPowerValue)) {
        Serial.printf("Rejected TP=%d (valid range %d-%d dBm)\n", newTxPowerValue, kMinTxPower, kMaxTxPower);
        return false;
    }

    bool changed = false;

    if (newSfValue != -1 && sf != newSfValue) {
        sf = newSfValue;
        changed = true;
    }

    if (newSbwValue != -1 && sbw != newSbwValue) {
        sbw = newSbwValue;
        changed = true;
    }

    if (newTxPowerValue != -1 && txPower != newTxPowerValue) {
        txPower = newTxPowerValue;
        changed = true;
    }

    if (changed) {
        updateFlag = 1;
        Serial.printf("New LoRa params queued -> SF: %d, BW: %ld Hz, TP: %d dBm\n", sf, sbw, txPower);
    }

    return changed;
}

bool sendConfigAck(int ackSf, long ackSbw, int ackTxPower, uint16_t ackSeq) {
    bool anySent = false;
    String basePayload = "ACK sf=" + String(ackSf) + " sbw=" + String(ackSbw) + " tp=" + String(ackTxPower) + " seq=" + String(ackSeq);
    uint16_t crc = crc16Ccitt(reinterpret_cast<const uint8_t *>(basePayload.c_str()), basePayload.length());
    String payload = basePayload + " crc=" + toHex4(crc);

    for (int i = 0; i < kAckRepeatCount; ++i) {
        if (LoRa.beginPacket() == 0) {
            Serial.println("Failed to start ACK packet.");
            delay(kAckRepeatDelayMs);
            continue;
        }

        LoRa.print(payload);

        int result = LoRa.endPacket();  // blocking to ensure TX completes before updating params
        if (result == 1) {
            anySent = true;
            Serial.printf("ACK sent (%d/%d) -> SF: %d, BW: %ld Hz, TP: %d dBm, SEQ: %u\n",
                          i + 1, kAckRepeatCount, ackSf, ackSbw, ackTxPower, ackSeq);
        } else {
            Serial.printf("ACK send failed (code %d).\n", result);
        }

        delay(kAckRepeatDelayMs);
    }

    // Return to receive mode after sending ACKs
    LoRa.receive();
    return anySent;
}

void applyLoRaConfig() {

    if (newSf != sf || newSbw != sbw || newTxPower != txPower || updateFlag == 1) {
        updateFlag = 0;
        newSf = sf;
        newSbw = sbw;
        newTxPower = txPower;
        LoRa.setSignalBandwidth(sbw);
        LoRa.setSpreadingFactor(sf);
        LoRa.setTxPower(txPower, PA_OUTPUT_PA_BOOST_PIN);
        Serial.printf("LoRa params updated -> SF: %d, BW: %ld Hz, TP: %d dBm\n", sf, sbw, txPower);
    }
}

void setupLoRa() {

    Serial.println("LoRa Receiver");

    SPI.begin(SCK, MISO, MOSI, SS);
    // Setup LoRa transceiver module pins
    LoRa.setPins(SS, RST, DIO0);

    //869.400-869.650 MHz for 10% duty cycle
    while (!LoRa.begin(869.400E6)) {
        Serial.print(".");
        delay(500);
    }

    // Change sync word (0xF3) to match the receiver
    // The sync word assures you don't get LoRa messages from other LoRa transceivers
    // ranges from 0-0xFF
    LoRa.setSyncWord(0xF3);
    LoRa.setSignalBandwidth(sbw);
    LoRa.setSpreadingFactor(sf);
    LoRa.setTxPower(txPower, PA_OUTPUT_PA_BOOST_PIN);
    Serial.println("LoRa Initializing OK!");

    applyLoRaConfig();
    
    // Start in receive mode
    LoRa.receive();
}