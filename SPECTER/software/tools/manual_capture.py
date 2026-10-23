"""
SPECTER Phase 1 - manual 3D capture.

Tilt the lidar head by hand to a measured angle (phone inclinometer),
type that angle, and the script grabs revolutions at that position.
Repeat for each angle, then type q to finish.

Saves two files:
  scan_<time>.ply      x, y, z, intensity  -> open in CloudCompare
  scan_<time>_raw.csv  angle, bearing, range, quality
                       -> rebuild the cloud later with a corrected offset,
                          without rescanning:
                          python manual_capture.py scan_<time>_raw.csv
"""
import sys
import time

import numpy as np
import serial
from plyfile import PlyData, PlyElement

# ---------------------------------------------------------------- settings
PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_bad54130f45df01185735a401045c30f-if00-port0'
BAUD = 460800
REVS_PER_ANGLE = 3      # revolutions captured at each angle

# Perpendicular distance from the shaft's centreline to the lidar's scan
# plane, in metres. Measure this in your CAD model. Sign: positive if the
# scan plane sits above the shaft when the head is level.
OFFSET_M = 0.040

# Bearing of the lidar's own 0 deg direction relative to the shaft axis.
# 0 if the triangle mark on top of the C1 points along the shaft.
PHI_OFFSET_DEG = 0.0

# ---------------------------------------------------------------- protocol
CMD_STOP = b'\xA5\x25'
CMD_SCAN = b'\xA5\x20'
MOTOR_ON = b'\xA5\xF0\x02\x94\x02\xC1'
MOTOR_OFF = b'\xA5\xF0\x02\x00\x00\x57'
SCAN_HEADER = b'\xA5\x5A\x05\x00\x00\x40\x81'


def start_scan(s):
    s.dtr = False
    s.write(CMD_STOP)
    time.sleep(0.5)
    s.reset_input_buffer()
    s.write(MOTOR_ON)
    time.sleep(3)
    s.write(CMD_SCAN)
    header = s.read(7)
    if header != SCAN_HEADER:
        raise RuntimeError(f'unexpected scan header: {header.hex() or "nothing"}')


def stop_scan(s):
    try:
        s.write(CMD_STOP)
        time.sleep(0.1)
        s.write(MOTOR_OFF)
        time.sleep(0.1)
        s.dtr = True
    except Exception:
        pass


def revolutions(s):
    """Yield revolutions as lists of (quality, bearing_deg, dist_mm)."""
    rev = []
    buf = bytearray(s.read(5))
    while True:
        if len(buf) < 5:
            raise RuntimeError('sensor stopped sending data')
        start = buf[0] & 1
        start_inv = (buf[0] >> 1) & 1
        if start == start_inv or (buf[1] & 1) != 1:
            buf = buf[1:] + s.read(1)
            continue
        quality = buf[0] >> 2
        bearing = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0
        dist = (buf[3] | (buf[4] << 8)) / 4.0
        if start and rev:
            yield rev
            rev = []
        rev.append((quality, bearing, dist))
        buf = bytearray(s.read(5))


def grab(s, n):
    """Discard stale data, then return n complete revolutions."""
    s.reset_input_buffer()
    gen = revolutions(s)
    next(gen)                       # first revolution is partial - skip it
    samples = []
    for _ in range(n):
        samples += next(gen)
    return samples


def to_xyz(theta_deg, phi_deg, r_m, d):
    """Nod axis = x. Sensor offset d perpendicular to the axis."""
    th = np.radians(theta_deg)
    ph = np.radians(phi_deg + PHI_OFFSET_DEG)
    x = r_m * np.cos(ph)
    y = r_m * np.sin(ph) * np.cos(th) - d * np.sin(th)
    z = r_m * np.sin(ph) * np.sin(th) + d * np.cos(th)
    return x, y, z


def save_ply(path, x, y, z, intensity):
    v = np.empty(len(x), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                ('intensity', 'f4')])
    v['x'], v['y'], v['z'], v['intensity'] = x, y, z, intensity
    PlyData([PlyElement.describe(v, 'vertex')]).write(path)


def build_cloud(arr, ply_path):
    x, y, z = to_xyz(arr[:, 0], arr[:, 1], arr[:, 2], OFFSET_M)
    save_ply(ply_path, x, y, z, arr[:, 3])
    print(f'saved {ply_path}  ({len(arr):,} points, offset {OFFSET_M * 1000:.1f} mm)')


def rebuild(csv_path):
    arr = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    build_cloud(arr, csv_path.replace('_raw.csv', f'_d{OFFSET_M * 1000:.0f}mm.ply'))


def capture():
    rows = []                       # theta_deg, phi_deg, range_m, quality
    s = serial.Serial(PORT, BAUD, timeout=5)
    try:
        start_scan(s)
        print('lidar running.')
        while True:
            a = input('\nset the head, hold it still, type its angle in degrees '
                      '(q to finish): ').strip()
            if a.lower() == 'q':
                break
            try:
                theta = float(a)
            except ValueError:
                print('  not a number, try again')
                continue
            samples = grab(s, REVS_PER_ANGLE)
            valid = [(theta, phi, d / 1000.0, q)
                     for q, phi, d in samples if q > 0 and d > 0]
            rows += valid
            print(f'  {theta:+.1f} deg: {len(valid)} points  (total {len(rows):,})')
    finally:
        stop_scan(s)
        s.close()

    if not rows:
        print('nothing captured')
        return

    arr = np.array(rows)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    np.savetxt(f'scan_{stamp}_raw.csv', arr, delimiter=',',
               header='theta_deg,phi_deg,range_m,quality', comments='')
    print(f'saved scan_{stamp}_raw.csv')
    build_cloud(arr, f'scan_{stamp}.ply')


if __name__ == '__main__':
    if len(sys.argv) > 1:
        rebuild(sys.argv[1])
    else:
        capture()
