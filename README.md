# LoRa Telemetry Field Test Workspace

This workspace contains the code and collected data for LoRa telemetry field tests. The main system is a sender/receiver pair that transmits CAN telemetry over LoRa while sweeping radio parameters, then logs receiver-side measurements into CSV files for later analysis.

The Arduino CAN generator is included as a support tool. It creates repeatable CAN traffic so the LoRa sender can be tested without the full vehicle telemetry stack connected.

## System Overview

```text
Arduino_CAN_generator  ->  CAN bus  ->  LoRa_sender  ->  LoRa link  ->  LoRa_receiver  ->  serial_reader.py  ->  test_data CSVs
```

- `Arduino_CAN_generator/` simulates vehicle CAN frames at 500 kbps.
- `LoRa_sender/` reads CAN frames, formats them as LoRa payloads, and runs the field-test sweep.
- `LoRa_receiver/` receives LoRa packets, reports RSSI values, applies RSSI filters, and ACKs parameter changes.
- `serial_reader.py` reads receiver serial output and writes CSV files into `test_data/`.
- `test_data/` contains the field-test dataset grouped by measured distance.

## Field-Test Workflow

1. Flash the receiver sketch from `LoRa_receiver/` to the receiver board.
2. Flash the sender sketch from `LoRa_sender/` to the sender board.
3. Optional: flash `Arduino_CAN_generator/` to an Arduino UNO R4 Minima and connect it to the sender CAN bus.
4. Start the Python logger on the computer connected to the receiver:

```bash
python serial_reader.py --list-ports
python serial_reader.py --port <PORT> --distance <DISTANCE_METERS>
```

5. Open the sender serial monitor at `250000 baud`.
6. Run:

```text
starttest
```

The sender cycles through valid combinations of spreading factor, bandwidth, and transmit power. For each configuration it sends a `CFG` packet, waits for the receiver to ACK it, applies the same local LoRa settings, then sends telemetry packets. The receiver prints one `DATA|...` line per received packet, and `serial_reader.py` stores those rows in a distance/config-specific CSV file.

## Test Parameters

The sender test sequence is defined in `LoRa_sender/LoRa_send_test.ino`.

| Parameter | Values |
| --- | --- |
| Spreading factor | `7`, `8`, `9`, `10`, `11`, `12` |
| Signal bandwidth | `62500`, `125000`, `250000`, `500000` Hz |
| TX power | `2`, `12`, `22` dBm |
| Packets per config | `100` |
| Frequency | `869.400 MHz` |
| Sync word | `0xF3` |

Configurations whose estimated transmit interval is above the configured limit are skipped by the sender.

## Dataset Layout

CSV files are stored under `test_data/distance_<distance>m/` and named by radio configuration:

```text
test_data/distance_6.25m/SF7_BW62500_TP12.csv
```

Each CSV row contains:

```text
timestamp,rssi,kalman_rssi,sma_rssi,time_between_messages_ms,payload
```

The payload is either a configuration message, such as `CFG sf=7 sbw=62500 tp=12`, or a forwarded CAN frame in this format:

```text
sender_millis,can_id,dlc,byte0,byte1,...
```

## Setup Commands

Install dependencies on your machine before building or logging.

Arduino CLI board support and libraries:

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli core install arduino:renesas_uno
arduino-cli lib install LoRa
arduino-cli lib install ESP32-TWAI-CAN
arduino-cli lib install Arduino_CAN
```

Python logger dependencies:

```bash
python -m pip install pyserial PyQt5
```

## Directory Guides

- [`LoRa_sender/README.md`](LoRa_sender/README.md) documents the sender firmware, test commands, CAN input, and LoRa sweep behavior.
- [`LoRa_receiver/README.md`](LoRa_receiver/README.md) documents the receiver firmware, RSSI output, ACK handling, and serial logger format.
- [`Arduino_CAN_generator/README.md`](Arduino_CAN_generator/README.md) documents the CAN simulator and CAN frame layout.
- [`test_data/README.md`](test_data/README.md) documents the dataset structure and CSV columns.

