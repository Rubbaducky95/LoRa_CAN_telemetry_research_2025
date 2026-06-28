# LoRa Sender

`LoRa_sender` is the active field-test transmitter. It reads CAN frames from the ESP32 TWAI/CAN interface, formats each frame as a compact text payload, and sends it over LoRa while sweeping radio settings.

## Responsibilities

- Configure the LoRa transceiver at `869.400 MHz` with sync word `0xF3`.
- Read CAN frames at `500 kbps`.
- Forward CAN payloads over LoRa during a test run.
- Coordinate radio parameter changes with the receiver using `CFG` messages and CRC-protected `ACK` responses.
- Enforce a 10% duty-cycle wait based on measured transmit time.
- Run an automated field-test sweep using serial commands.

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

CAN pins are defined in `CAN_functions.h`:

| Signal | Pin |
| --- | --- |
| CAN TX | `16` |
| CAN RX | `4` |

The sender expects a CAN transceiver connected to those CAN pins and a LoRa module connected to the SPI pins above.

## Serial Interface

Open the sender serial monitor at `250000 baud`.

Commands:

| Command | Purpose |
| --- | --- |
| `starttest` | Start the automated field-test sweep |
| `stoptest` | Stop the current test |
| `skipconfig` | Skip the current radio configuration |
| `teststatus` | Print current test progress |
| `status` | Print active LoRa parameters |
| `sf=<7-12>` | Manually set spreading factor after receiver ACK |
| `sbw=<Hz>` | Manually set signal bandwidth after receiver ACK |
| `tp=<2-22>` | Manually set transmit power after receiver ACK |
| `help` | Print command help |

## Automated Test Sweep

The test sweep is configured in `LoRa_send_test.ino`:

| Parameter | Values |
| --- | --- |
| Spreading factor | `7`, `8`, `9`, `10`, `11`, `12` |
| Signal bandwidth | `62500`, `125000`, `250000`, `500000` Hz |
| TX power | `2`, `12`, `22` dBm |
| Packets per config | `100` |

For each valid configuration, the sender:

1. Sends `CFG sf=<sf> sbw=<bandwidth> tp=<power> seq=<seq> crc=<crc>`.
2. Waits until the receiver returns a matching `ACK`.
3. Applies the new local LoRa configuration.
4. Sends up to 100 CAN-derived LoRa packets.
5. Advances to the next valid configuration.

## LoRa Payload Format

CAN frames are sent as comma-separated text:

```text
sender_millis,can_id,dlc,byte0,byte1,byte2,...
```

Example:

```text
257342,100,8,E9,E0,21,40,6E,FA,D7,41
```

## Build Notes

Install the ESP32 board package and libraries on your development machine:

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install LoRa
arduino-cli lib install ESP32-TWAI-CAN
```

Then compile/upload using the FQBN and serial port for your ESP32-S3 board.

