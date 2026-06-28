# Field-Test Dataset

`test_data` contains CSV files captured during LoRa telemetry field tests.

## Directory Structure

Data is grouped by test distance:

```text
test_data/distance_6.25m/
test_data/distance_12.5m/
test_data/distance_18.75m/
...
```

CSV filenames encode the radio configuration:

```text
SF7_BW62500_TP12.csv
```

This means:

- `SF7`: spreading factor 7
- `BW62500`: signal bandwidth 62500 Hz
- `TP12`: transmit power 12 dBm

## CSV Columns

Each file uses this header:

```text
timestamp,rssi,kalman_rssi,sma_rssi,time_between_messages_ms,payload
```

| Column | Description |
| --- | --- |
| `timestamp` | Computer timestamp when `serial_reader.py` processed the receiver line |
| `rssi` | Raw packet RSSI from the receiver |
| `kalman_rssi` | Receiver-side Kalman-filtered RSSI |
| `sma_rssi` | Receiver-side simple moving average RSSI |
| `time_between_messages_ms` | Time since the previous logged message in the same CSV |
| `payload` | Received LoRa payload |

Payload values can be configuration messages:

```text
CFG sf=7 sbw=62500 tp=12
```

or forwarded CAN frames:

```text
257342,100,8,E9,E0,21,40,6E,FA,D7,41
```

The forwarded CAN format is:

```text
sender_millis,can_id,dlc,byte0,byte1,byte2,...
```

## Logging New Data

Use the Python serial reader from the workspace root:

```bash
python serial_reader.py --list-ports
python serial_reader.py --port <PORT> --distance <DISTANCE_METERS>
```

The logger creates new CSV files when it detects the receiver applying a new sender configuration. If a target CSV already exists, it skips that file to avoid overwriting existing field data.

