# Scans

Full-size scans (tens of MB each) are not stored in git. They are attached to the repository Releases page:
**[link to the Release]**

`sample/` holds one small scan you can open straight away. To make one under 10 MB, open a scan in CloudCompare, use Edit > Subsample (Space, 0.02 m), and save as LAZ.

Each scan has three files:
- `scan_<time>.laz` LAS 1.2, compressed, 1 mm coordinate precision, intensity and grey colour
- `scan_<time>.ply` the same cloud for CloudCompare
- `scan_<time>_raw.csv` nod angle, bearing, range, quality; rebuild with new calibration using `software/specter_capture.py`
