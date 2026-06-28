# LoRa Receiver

`LoRa_receiver` is the field-test receiver. It listens for LoRa telemetry packets, measures RSSI, applies two RSSI filters, prints machine-readable serial output, and acknowledges sender configuration changes.

## Responsibilities

- Configure the LoRa transceiver at `869.400 MHz` with sync word `0xF3`.
- Receive telemetry packets from `LoRa_sender`.
- Process `CFG` messages from the sender and return CRC-protected ACK packets.
- Keep sender and receiver radio parameters synchronized.
- Print received payload, raw RSSI, Kalman-filtered RSSI, and simple moving average RSSI.

## Hardware Connections

LoRa pins are defined in `LoRa_functions.h`:

| Signal | Pin |
| --- | --- |
| `SS` | `37` |
| `RST` | `3` |
| `DIO0` | `48` |
| `SCK` | `38` |
| `MISO` | `39` |
| `MOSI` | `1` |

The receiver starts with default LoRa settings:

| Setting | Default |
| --- | --- |
| Frequency | `869.400 MHz` |
| Sync word | `0xF3` |
| Spreading factor | `7` |
| Bandwidth | `62500` Hz |
| TX power | `10` dBm |

## Serial Output

Open the receiver serial monitor at `115200 baud`.

Each received LoRa packet is printed on one line:

```text
DATA|LoRa_data:<payload>|RSSI:<raw_rssi>|Kalman_RSSI:<filtered_rssi>|SMA_RSSI:<moving_average_rssi>
```

Example:

```text
DATA|LoRa_data:257342,100,8,E9,E0,21,40,6E,FA,D7,41|RSSI:-51.0000|Kalman_RSSI:-59.2074|SMA_RSSI:-58.2000
```

This is the format consumed by `serial_reader.py`.

## Configuration Sync

The sender changes LoRa parameters by transmitting a `CFG` message:

```text
CFG sf=7 sbw=62500 tp=12 seq=1 crc=ABCD
```

The receiver validates the spreading factor, bandwidth, transmit power, sequence number, and CRC. If valid, it sends repeated ACK packets and applies the new radio parameters:

```text
ACK sf=7 sbw=62500 tp=12 seq=1 crc=1234
```

Duplicate sequence numbers with already-active parameters are acknowledged again so the sender can recover from missed ACKs.

## Local Commands

Commands are available over the receiver serial monitor:

| Command | Purpose |
| --- | --- |
| `status` | Print current LoRa parameters |
| `sf=<7-12>` | Manually set spreading factor |
| `sbw=<Hz>` | Manually set signal bandwidth |
| `tp=<2-22>` | Manually set transmit power |
| `help` | Print command help |

During field tests, normally let the sender drive parameter changes so the dataset stays aligned with the sender configuration.

## Build Notes

Install the ESP32 board package and LoRa library on your development machine:

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install LoRa
```

Then compile/upload using the FQBN and serial port for your ESP32-S3 board.

