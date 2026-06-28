# Arduino CAN Generator

`Arduino_CAN_generator` is a support sketch for field testing. It generates simulated vehicle telemetry on a CAN bus so the LoRa sender can be exercised without the real vehicle electronics connected.

## What It Does

- Runs on an Arduino UNO R4 Minima.
- Initializes CAN at `500 kbps`.
- Sends simulated CAN frames every `100 ms` (`10 Hz`).
- Generates changing values for motor current, velocity, distance, battery voltage/current, temperatures, and MPPT power.
- Uses fixed UNO R4 Minima CAN pins:
  - `D4` / `CANTX0` for CAN TX
  - `D5` / `CANRX0` for CAN RX

## Hardware

Required hardware:

- Arduino UNO R4 Minima
- CAN transceiver module
- CAN bus wiring to the LoRa sender CAN transceiver
- Proper CAN termination, normally `120 ohm` at each end of the bus

Basic wiring:

| Arduino UNO R4 Minima | CAN transceiver |
| --- | --- |
| `D4` / `CANTX0` | `TXD` |
| `D5` / `CANRX0` | `RXD` |
| `5V` | `VCC` |
| `GND` | `GND` |

Connect the transceiver `CANH` and `CANL` pins to the sender-side CAN bus.

## Build Notes

Install the UNO R4 board package and CAN library on your development machine:

```bash
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno
arduino-cli lib install Arduino_CAN
```

Compile:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:minima Arduino_CAN_generator
```

Upload, replacing `<PORT>` with the Arduino serial port:

```bash
arduino-cli upload -p <PORT> --fqbn arduino:renesas_uno:minima Arduino_CAN_generator
```

## Serial Monitor

Open the serial monitor at `115200 baud`.

On startup, the sketch prints the CAN pin configuration and whether CAN initialization succeeded.

## CAN Message Layout

All messages use 8-byte CAN payloads. Multi-byte integer values are little-endian. Floating point values are sent as 32-bit Arduino `float` values.

| CAN ID | Data | Payload layout |
| --- | --- | --- |
| `0x200` | MPPT1 wattage | bytes `4-7`: `float` power in W |
| `0x210` | MPPT2 wattage | bytes `4-7`: `float` power in W |
| `0x220` | MPPT3 wattage | bytes `4-7`: `float` power in W |
| `0x402` | Motor current | bytes `4-7`: `float` current in A |
| `0x403` | Vehicle velocity | bytes `4-7`: `float` velocity in km/h |
| `0x40B` | Motor and controller temperature | bytes `0-3`: motor temperature `float`, bytes `4-7`: motor controller temperature `float` |
| `0x40E` | Distance travelled | bytes `0-3`: `float` distance |
| `0x601` | Battery temperatures and cell IDs | byte `1`: BMS temp, byte `2`: high cell temp, byte `3`: low cell temp, byte `4`: average cell temp, byte `5`: high-temp cell ID, byte `6`: low-temp cell ID |
| `0x602` | Battery current, voltage, and cell voltages | bytes `0-1`: signed current x10, bytes `2-3`: voltage x10, bytes `4-5`: low cell voltage x10000, bytes `6-7`: high cell voltage x10000 |
| `0x603` | Average cell voltage | bytes `0-1`: average cell voltage x10000 |

## Changing the Simulated Data

The generated values are stored in the `VehicleData` struct in `Arduino_CAN_generator.ino`. The function `updateSimulatedData()` updates those values before each transmit cycle.

To use real sensors instead of simulated values, replace the calculations in `updateSimulatedData()` with sensor reads and assign the results to `vehicleData`.

