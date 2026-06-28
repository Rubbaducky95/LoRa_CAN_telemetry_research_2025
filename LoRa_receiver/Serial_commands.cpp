#include <Arduino.h>

#include "Serial_commands.h"
#include "LoRa_functions.h"

namespace {

void printParameterHelp() {
    Serial.println("Commands:");
    Serial.println("  sf=<7-12>         -> set spreading factor");
    Serial.println("  sbw=<Hz>          -> set bandwidth (e.g. 15600, 20800, 62500, 250000)");
    Serial.println("  tp=<2-22>         -> set transmit power (dBm, PA_BOOST)");
    Serial.println("  status            -> print current settings");
    Serial.println("  help              -> show this message");
}

}  // namespace

void serialSetup() {
    delay(3000);
    Serial.begin(115200);
    while (!Serial)
        ;
    printParameterHelp();
}

void handleSerialCommands() {
    if (!Serial.available()) return;

    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) return;

    String lower = command;
    lower.toLowerCase();

    if (lower == "help") {
        printParameterHelp();
        return;
    }

    if (lower == "status") {
        Serial.printf("Current LoRa params -> SF: %d, BW: %ld Hz, TP: %d dBm\n", sf, sbw, txPower);
        return;
    }

    if (lower.startsWith("sf=")) {
        int newValue = lower.substring(3).toInt();
        if (!isValidSpreadingFactor(newValue)) {
            Serial.println("Invalid SF. Choose between 7 and 12.");
            return;
        }
        if (queueLoRaParams(newValue, -1, -1)) {
            applyLoRaConfig();
        } else {
            Serial.println("SF unchanged.");
        }
        return;
    }

    if (lower.startsWith("sbw=")) {
        long newValue = lower.substring(4).toInt();
        if (!isValidBandwidth(newValue)) {
            Serial.println("Invalid bandwidth. Try one of: 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000.");
            return;
        }
        if (queueLoRaParams(-1, newValue, -1)) {
            applyLoRaConfig();
        } else {
            Serial.println("Bandwidth unchanged.");
        }
        return;
    }

    if (lower.startsWith("tp=")) {
        int newValue = lower.substring(3).toInt();
        if (!isValidTransmitPower(newValue)) {
            Serial.printf("Invalid TX power. Choose between %d and %d dBm.\n", 2, 22);
            return;
        }
        if (queueLoRaParams(-1, -1, newValue)) {
            applyLoRaConfig();
        } else {
            Serial.println("Transmit power unchanged.");
        }
        return;
    }

    Serial.println("Unknown command. Type 'help' for options.");
}

