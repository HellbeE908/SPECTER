# Wiring

| Signal | From | To | Note |
|---|---|---|---|
| EN | XIAO D0 | TMC2209 EN | Low = enabled |
| STEP | XIAO D1 | TMC2209 STEP | Trapezoidal ramp |
| DIR | XIAO D2 | TMC2209 DIR | Positive = clockwise (rear view) |
| VIO / GND | XIAO 3V3 / GND | Driver VIO / GND | Common ground with pack negative |
| VM | 11.1 V rail | TMC2209 VM | 100 µF + 100 nF at the pins |
| Coils | Driver 1A/1B, 2A/2B | NEMA 17 | Pairs found by resistance, 2-5 Ω |
| Lidar data | RPLIDAR C1 | Pi 5 USB | CP2102N, 460,800 baud |
| Control | XIAO ESP32-S3 | Pi 5 USB | USB CDC, 115,200 baud |

Power: 3S2P pack (9.0-12.6 V, BMS cut-off 8.7 V) -> 5 A fuse -> master switch -> (a) TMC2209 VM directly, (b) USB-PD module -> 5 V -> Raspberry Pi 5.

Driver: 1/8 microstepping (MS1/MS2 floating), V_ref 0.64 V.


