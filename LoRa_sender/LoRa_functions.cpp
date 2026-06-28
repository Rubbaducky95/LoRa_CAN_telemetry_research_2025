#include "LoRa_functions.h"

long sbw = 62500; // 20800, 62500, 125000, 250000 (default matches first test config)
int sf = 7;       // 6, 7, 8, 9, 10, 11, 12
int updateFlag = 1;
long newSbw = sbw;
int newSf = sf;
int txPower = 10;  // Safe default: 10 dBm (compliant with 3 dBi antenna)
int newTxPower = txPower;

void applyLoRaConfig() {
    if (newSf == sf && newSbw == sbw && newTxPower == txPower && updateFlag == 0) {
        return;
    }

    LoRa.setSpreadingFactor(sf);
    LoRa.setSignalBandwidth(sbw);
    if (txPower > 14) {
        LoRa.setTxPower(txPower, PA_OUTPUT_PA_BOOST_PIN);
    } else {
        LoRa.setTxPower(txPower, PA_OUTPUT_RFO_PIN);
    }
    
    newSf = sf;
    newSbw = sbw;
    newTxPower = txPower;
    updateFlag = 0;

    Serial.printf("Applied LoRa config -> SF: %d, BW: %ld Hz, TP: %d dBm\n", sf, sbw, txPower);
}

void setupLoRa() {

    Serial.println("Initializing LoRa transceiver...");

    SPI.begin(SCK, MISO, MOSI, SS);
    // Setup LoRa transceiver module pins
    LoRa.setPins(SS, RST, DIO0);

    //869.400-869.650 MHz for 10% duty cycle (Sweden sub-band)
    while (!LoRa.begin(869.400E6)) {
        Serial.println(".");
        delay(500);
    }

    // Change sync word (0xF3) to match the receiver
    // The sync word assures you don't get LoRa messages from other LoRa transceivers
    // ranges from 0-0xFF
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initializing OK!");

    applyLoRaConfig();
}