#ifndef LORA_FUNCTIONS_H
#define LORA_FUNCTIONS_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define SS 37 // change back to 5
#define RST 3 // change back to 14
#define DIO0 48 // change back to 2

// Define pins for miso mosi and clock
#define SCK 38
#define MISO 39
#define MOSI 1

// Variables
extern long sbw; 
extern int sf;
extern int updateFlag;
extern long newSbw;
extern int newSf;
extern int txPower;
extern int newTxPower;

void setupLoRa();
void applyLoRaConfig();
bool isValidBandwidth(long candidate);
bool isValidSpreadingFactor(int candidate);
bool isValidTransmitPower(int candidate);
bool queueLoRaParams(int newSfValue, long newSbwValue, int newTxPowerValue);
uint16_t crc16Ccitt(const uint8_t *data, size_t length);
bool sendConfigAck(int ackSf, long ackSbw, int ackTxPower, uint16_t ackSeq);

#endif