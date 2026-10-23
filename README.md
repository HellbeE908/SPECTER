# SPECTER

**A low-cost terrestrial laser scanner for 3D documentation of urban infrastructure.**

Vishwakarma Awards 2026-27 · Application ID **VK26-IITH-SC-0290** · Theme 6, Sustainable Cities — Smart Urban Infrastructure \& Asset Tracking

SPECTER park survey: 4 stations merged, 1.2 million points](media/park\_survey\_1.2M.jpg)

SPECTER nods a 360° direct time-of-flight 2D LiDAR about an offset axis and turns each sweep into a survey-format 3D point cloud. The complete unit costs **₹41,256**, against **₹25–80 lakh** for commercial terrestrial scanners.

## Results so far

|Result|Value|Status|
|-|-|-|
|Valid returns per revolution|461|Measured|
|Single-station room scan|250,000 points|Measured|
|Outdoor survey, 4 merged stations|1.2 million points|Measured|
|Nod-axis angular error|±0.11°, non-accumulating|Measured|
|Pi 5 supply under full load|4.81 V, no throttling|Measured|
|Range accuracy|±30 mm|Manufacturer spec; validation underway|

!\[Single room station, 250,000 points, front walls cut away](media/room\_scan\_250k.jpg)

## How it works

```
3S Li-ion pack ─ 5 A fuse ─ switch ─┬─ TMC2209 (12 V) ─ NEMA 17 ─ 3:1 GT2 belt ─ 8 mm spindle ─ RPLIDAR C1
                                    └─ USB-PD module (5 V) ─ Raspberry Pi 5
Raspberry Pi 5 ─USB─ RPLIDAR C1        (scan data, 460,800 baud)
Raspberry Pi 5 ─USB─ XIAO ESP32-S3     (motion commands, 115,200 baud) ─ EN/STEP/DIR ─ TMC2209
```

* **Motion** runs on the XIAO ESP32-S3, because Linux can pause a process for milliseconds and silently drop stepper pulses. Every move is computed from an absolute target angle, so error never accumulates.
* **Capture** runs on the Pi 5: move, wait for the controller to confirm the angle, discard the partial revolution, record full revolutions, repeat.
* **Geometry:** the C1 sits 50 mm off the nod axis, so no part of the scanner ever enters the scan plane. The offset was calibrated from scan data.
* **Output:** PLY (for CloudCompare) and LAS/LAZ at 1 mm coordinate precision (for survey and GIS tools), plus the raw angles and ranges, so any scan can be rebuilt with new calibration values.

## Bill of materials

 Main items: RPLIDAR C1 ₹7,300 · Raspberry Pi 5 (8 GB) ₹20,000 · tripod ₹4,500 · 3S2P battery ₹1,679 · XIAO ESP32-S3 ₹1,000 · NEMA 17 + TMC2209 ₹1,619. **Total ₹41,256.**

The Raspberry Pi is priced at October 2026 levels, after memory-driven price rises; a lower-cost compute board is the most direct way to reduce unit cost.

## Quick start

**1. Flash the firmware.** Open `firmware/specter\_nod/specter\_nod.ino` in Arduino IDE. Board: `XIAO\_ESP32S3`, USB CDC On Boot: Enabled. Upload.

**2. Set up the Pi** (Raspberry Pi OS Lite, 64-bit):

```bash
python3 -m venv \~/specter-env
source \~/specter-env/bin/activate
pip install -r requirements.txt
```

**3. Scan.** Connect the C1 and the XIAO to the Pi by USB (data cables). The script finds both automatically.

```bash
python software/specter\_capture.py -100 100 0.375
```

Level the head when prompted, press Enter, and the sweep runs (about 4 minutes for roughly 500,000 points indoors). Press Ctrl+C at any time to stop: the head returns to level and the data captured so far is saved.

**4. View.** Open the `.laz` or `.ply` in [CloudCompare](https://www.cloudcompare.org), set colours to the intensity scalar field (Grey scale), and turn on Eye-Dome Lighting.

**Rebuild a scan with new calibration values** (no rescan needed):

```bash
python software/specter\_capture.py scan\_YYYYMMDD\_HHMMSS\_raw.csv
```

## Safety

* Never connect or disconnect motor wires while the battery is on: the inductive spike destroys the driver.
* Shut the Pi down before switching off the battery, to avoid corrupting the SD card.
* The RPLIDAR C1 is a Class 1 laser product (manufacturer rating).

## Timeline

|When|Milestone|
|-|-|
|Sep 2026|Mechanism and electronics designed; Stage 1 proposal submitted|
|Early Oct 2026|Custom C1 driver written; firmware moved to absolute positioning, eliminating 3.6° drift|
|8 Oct 2026|First 3D scans; 50 mm sensor offset calibrated from scan data|
|Mid Oct 2026|Dense scans: 125,000 and 250,000 points per station|
|20 Oct 2026|Outdoor park survey: 4 stations merged into 1.2 million points|
|Oct 2026|Enclosure designed; Stage 2 presentation|

**Next:** published accuracy figures, automatic multi-station registration, change detection, and phone-controlled standalone operation, ahead of the Dec 2026 – Feb 2027 semi-finals and finals.

## Team

Rahul Gupta, Parshant Dalal, Dronacharya College of Engineering, faculty mentor - None

## License

Software and firmware: MIT (see `LICENSE`). Hardware designs and documentation: CC BY 4.0.

