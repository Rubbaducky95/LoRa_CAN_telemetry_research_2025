#include <Arduino.h>

#include "Serial_commands.h"
#include "LoRa_functions.h"

namespace {

const long kValidBandwidths[] = {15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};
const int kMinTxPower = 2;
const int kMaxTxPower = 22;

bool isValidBandwidth(long candidate) {
    for (long value : kValidBandwidths) {
        if (candidate == value) {
            return true;
        }
    }
    return false;
}

void printParameterHelp() {
    Serial.println("Commands:");
    Serial.println("  sf=<7-12>         -> set spreading factor");
    Serial.println("  sbw=<Hz>           -> set bandwidth (e.g. 15600, 20800, 62500, 250000)");
    Serial.println("  tp=<2-22>          -> set transmit power (dBm, PA_BOOST)");
    Serial.println("  status             -> print current settings");
    Serial.println("  starttest         -> start test sequence");
    Serial.println("  stoptest          -> stop current test");
    Serial.println("  skipconfig        -> skip current config and jump to next");
    Serial.println("  teststatus        -> show test progress");
    Serial.println("  help              -> show this message");
}

void printStatus() {
    Serial.printf("Current LoRa params -> SF: %d, BW: %ld Hz, TP: %d dBm\n", sf, sbw, txPower);
}

}  // namespace

void serialSetup() {
    delay(3000);
    Serial.begin(250000);
    while (!Serial)
        ;
    printParameterHelp();
    printStatus();
}

bool handleSerialCommands(String *unhandledCommand, SerialParameterChangeCallback onChange) {
    if (!Serial.available()) {
        return false;
    }

    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) {
        return false;
    }

    String lower = command;
    lower.toLowerCase();

    if (lower == "help") {
        printParameterHelp();
        return true;
    }

    if (lower == "status") {
        printStatus();
        return true;
    }

    // Test control commands
    if (lower == "starttest") {
        extern void startTest();
        startTest();
        return true;
    }

    if (lower == "stoptest") {
        extern void stopTest();
        stopTest();
        return true;
    }

    if (lower == "skipconfig") {
        extern void skipConfig();
        skipConfig();
        return true;
    }

    if (lower == "teststatus") {
        extern void printTestStatus();
        printTestStatus();
        return true;
    }

    if (lower.startsWith("sf=")) {
        int newValue = lower.substring(3).toInt();
        if (newValue < 7 || newValue > 12) {
            Serial.println("Invalid SF. Choose between 7 and 12.");
            return true;
        }
        int previousValue = sf;
        if (onChange != nullptr && !onChange("sf", newValue)) {
            Serial.println("SF update aborted (no ACK).");
            sf = previousValue;
            return true;
        }
        sf = newValue;
        updateFlag = 1;
        applyLoRaConfig();
        printStatus();
        return true;
    }

    if (lower.startsWith("sbw=")) {
        long newValue = lower.substring(4).toInt();
        if (!isValidBandwidth(newValue)) {
            Serial.println("Invalid bandwidth. Try one of: 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000.");
            return true;
        }
        long previousValue = sbw;
        if (onChange != nullptr && !onChange("sbw", newValue)) {
            Serial.println("Bandwidth update aborted (no ACK).");
            sbw = previousValue;
            return true;
        }
        sbw = newValue;
        updateFlag = 1;
        applyLoRaConfig();
        printStatus();
        return true;
    }

    if (lower.startsWith("tp=")) {
        int newValue = lower.substring(3).toInt();
        if (newValue < kMinTxPower || newValue > kMaxTxPower) {
            Serial.printf("Invalid TX power. Choose between %d and %d dBm.\n", kMinTxPower, kMaxTxPower);
            return true;
        }
        int previousValue = txPower;
        if (onChange != nullptr && !onChange("tp", newValue)) {
            Serial.println("Transmit power update aborted (no ACK).");
            txPower = previousValue;
            return true;
        }
        txPower = newValue;
        updateFlag = 1;
        applyLoRaConfig();
        printStatus();
        return true;
    }

    if (unhandledCommand != nullptr) {
        *unhandledCommand = command;
        return false;
    }

    Serial.println("Unknown command. Type 'help' for options.");
    return false;
}
