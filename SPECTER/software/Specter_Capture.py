"""
SPECTER - automated station capture.

Moves the nod axis through a sweep with the motor controller (XIAO ESP32-S3
or Nano ESP32, same firmware protocol), records complete lidar revolutions
at each angle, and saves:

  scan_<time>.ply      x, y, z, grey colour, intensity  -> CloudCompare
  scan_<time>.laz      same cloud as compressed LAS     -> GIS / survey tools
  scan_<time>_raw.csv  angle, bearing, range, quality
                       -> rebuild later with corrected offsets, no rescan:
                          python auto_capture_2.py scan_<time>_raw.csv

Press Ctrl+C during a scan to abort: the head returns to PARK_DEG and
everything captured so far is saved with "_partial" in the name.

Usage:
  python auto_capture_2.py               default sweep (~500k points, ~4 min)
  python auto_capture_2.py -100 100 1    start, stop, step in degrees
  python auto_capture_2.py file_raw.csv  rebuild a cloud from saved data

Coordinate frame (head level):
  x = forward, along the shaft, the way the C1's arrow points
  y = left
  z = up
  Positive nod = clockwise when viewed from behind, looking forward.
"""
import glob
import sys
import time

import numpy as np
import serial
from plyfile import PlyData, PlyElement

try:
    import laspy                     # pip install "laspy[lazrs]"
    HAVE_LASPY = True
except ImportError:
    HAVE_LASPY = False

# ================================================================ ports
# 'auto' finds each device in /dev/serial/by-id/. Paste a full path
# here instead if auto-detection ever picks the wrong one.
LIDAR_PORT = 'auto'
CONTROLLER_PORT = 'auto'
LIDAR_BAUD = 460800
CONTROLLER_BAUD = 115200

# ================================================================ sweep
START_DEG = -100.0
STOP_DEG = 100.0
STEP_DEG = 0.375            # 5 microsteps; with 2 revs ~500k points
REVS_PER_ANGLE = 2          # 2 roughly doubles points at little extra time
PARK_DEG = 0.0              # where the head returns after a scan or an abort

# ================================================================ geometry
# Vertical distance from the shaft centre line to the middle of the C1's
# spinning window, head level, in metres. Positive if the window is above.
OFFSET_M = 0.050
# Sideways distance from the shaft centre line to the C1's centre, metres.
# Positive to the left (seen from behind). 0 if the C1 sits over the shaft.
OFFSET_SIDE_M = 0.0
# Angle between the C1's 0 deg mark and the shaft axis (rule: 360 - reading
# for an object exactly on the shaft line). 0 if the arrow points along it.
PHI_OFFSET_DEG = 0.0
# C1 bearings increase clockwise seen from above (confirmed by test).
BEARING_CLOCKWISE = True

# ================================================================ lidar protocol
CMD_STOP = b'\xA5\x25'
CMD_SCAN = b'\xA5\x20'
MOTOR_ON = b'\xA5\xF0\x02\x94\x02\xC1'
MOTOR_OFF = b'\xA5\xF0\x02\x00\x00\x57'
SCAN_HEADER = b'\xA5\x5A\x05\x00\x00\x40\x81'

LIDAR_KEYS = ('silicon_labs', 'cp210')
CONTROLLER_KEYS = ('arduino', 'espressif', 'esp32', 'seeed', 'xiao')


# ================================================================ port discovery
def find_port(setting, include, exclude=()):
    if setting != 'auto':
        return setting
    ports = glob.glob('/dev/serial/by-id/*')
    for p in ports:
        name = p.lower()
        if any(k in name for k in include) and not any(k in name for k in exclude):
            return p
    raise RuntimeError(f'no device matching {include} found. '
                       f'Connected: {ports or "nothing"}')


# ================================================================ lidar
def lidar_start(s):
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


def lidar_stop(s):
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
            raise RuntimeError('lidar stopped sending data')
        start = buf[0] & 1
        start_inv = (buf[0] >> 1) & 1
        if start == start_inv or (buf[1] & 1) != 1:
            buf = buf[1:] + s.read(1)               # out of sync: slide a byte
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
    """Discard data collected while moving, then return n complete revolutions."""
    s.reset_input_buffer()
    gen = revolutions(s)
    next(gen)                                       # partial revolution
    samples = []
    for _ in range(n):
        samples += next(gen)
    return samples


# ================================================================ nod axis
def ctl_cmd(c_port, c, expect, timeout=15):
    """Send a command and wait for a reply starting with `expect`.
    Lines that don't match (e.g. a late reply to an earlier move) are skipped."""
    c_port.write((c + '\n').encode())
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = c_port.readline().decode(errors='replace').strip()
        if not line:
            continue
        if line.startswith(expect):
            return line
        if line.startswith('ERR'):
            raise RuntimeError(f'controller replied {line} to "{c}"')
    raise RuntimeError(f'no "{expect}" reply to "{c}"')


def goto(c_port, deg):
    reply = ctl_cmd(c_port, f'G{deg:.3f}', 'ARRIVED')
    return float(reply.split()[1])                  # actual angle reached


def park(c_port):
    """Let any interrupted move finish, then return the head to PARK_DEG."""
    # The controller only answers '?' once it has finished moving, so this
    # waits out a move that was in progress when Ctrl+C was pressed.
    ctl_cmd(c_port, '?', 'POS', timeout=20)
    print(f'returning head to {PARK_DEG:+.1f} deg...')
    reply = ctl_cmd(c_port, f'G{PARK_DEG:.3f}', 'ARRIVED', timeout=30)
    print(f'parked at {float(reply.split()[1]):+.2f} deg')


def level_and_zero(c_port):
    ctl_cmd(c_port, 'D', 'OK DISABLED')
    input('\nMotor released. Level the lidar by hand (phone inclinometer), '
          'then press Enter...')
    ctl_cmd(c_port, 'E', 'OK ENABLED')
    ctl_cmd(c_port, 'Z', 'OK ZERO')
    print('zero set.')


# ================================================================ geometry + output
def to_xyz(theta_deg, phi_deg, r_m):
    th = np.radians(theta_deg)
    ph = np.radians(phi_deg + PHI_OFFSET_DEG)
    if BEARING_CLOCKWISE:
        ph = -ph                                    # convert to anticlockwise
    # point in the sensor frame, shifted by the mounting offset
    xs = r_m * np.cos(ph)
    ys = r_m * np.sin(ph) + OFFSET_SIDE_M
    zs = OFFSET_M
    # rotate about the nod (x) axis
    x = xs
    y = ys * np.cos(th) - zs * np.sin(th)
    z = ys * np.sin(th) + zs * np.cos(th)
    return x, y, z


def grey_levels(intensity):
    """Map intensity to 0-1, stretched between the 2nd and 98th percentiles."""
    lo, hi = np.percentile(intensity, [2, 98])
    return np.clip((intensity - lo) / max(hi - lo, 1e-6), 0, 1)


def save_ply(path, x, y, z, intensity):
    """Grey colour from intensity (for direct viewing) plus raw intensity."""
    grey = (40 + 215 * grey_levels(intensity)).astype('u1')   # keep darkest visible
    v = np.empty(len(x), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
                                ('intensity', 'f4')])
    v['x'], v['y'], v['z'] = x, y, z
    v['red'] = v['green'] = v['blue'] = grey
    v['intensity'] = intensity
    PlyData([PlyElement.describe(v, 'vertex')]).write(path)


def save_las(path, x, y, z, intensity):
    """Compressed LAS 1.2, millimetre precision, 16-bit grey colour + intensity."""
    grey16 = ((40 + 215 * grey_levels(intensity)) * 257).astype(np.uint16)
    header = laspy.LasHeader(point_format=2, version='1.2')
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.floor([x.min(), y.min(), z.min()])
    las = laspy.LasData(header)
    las.x, las.y, las.z = x, y, z
    las.intensity = np.clip(intensity * 1000, 0, 65535).astype(np.uint16)
    las.red = grey16
    las.green = grey16
    las.blue = grey16
    las.write(path)


def build_cloud(arr, ply_path):
    x, y, z = to_xyz(arr[:, 0], arr[:, 1], arr[:, 2])
    save_ply(ply_path, x, y, z, arr[:, 3])
    print(f'saved {ply_path}  ({len(arr):,} points, offset {OFFSET_M * 1000:.1f} mm, '
          f'side {OFFSET_SIDE_M * 1000:.1f} mm, phi {PHI_OFFSET_DEG:+.1f} deg)')
    if HAVE_LASPY:
        las_path = ply_path[:-4] + '.laz'
        try:
            save_las(las_path, x, y, z, arr[:, 3])
            print(f'saved {las_path}')
        except Exception as e:
            print(f'LAS export failed ({e}) - the PLY is still fine')
    else:
        print('LAS export skipped: run  pip install "laspy[lazrs]"  to enable it')


def rebuild(csv_path):
    arr = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    tag = f'_d{OFFSET_M * 1000:.0f}_s{OFFSET_SIDE_M * 1000:.0f}_p{PHI_OFFSET_DEG:+.0f}'
    build_cloud(arr, csv_path.replace('_raw.csv', tag + '.ply'))


# ================================================================ main
def capture(start, stop, step):
    angles = np.arange(start, stop + step / 2, step)
    rows = []
    aborted = False

    lidar_path = find_port(LIDAR_PORT, LIDAR_KEYS)
    ctl_path = find_port(CONTROLLER_PORT, CONTROLLER_KEYS, exclude=LIDAR_KEYS)
    print(f'lidar      : {lidar_path}')
    print(f'controller : {ctl_path}')

    c_port = serial.Serial(ctl_path, CONTROLLER_BAUD, timeout=1)
    time.sleep(2)                                   # controller may reset on open
    c_port.reset_input_buffer()
    s = serial.Serial(lidar_path, LIDAR_BAUD, timeout=5)

    try:
        level_and_zero(c_port)
        print('starting lidar...   (Ctrl+C aborts, returns the head and keeps the data)')
        lidar_start(s)

        print(f'sweeping {start:+.1f} to {stop:+.1f} deg in {step:g} deg steps '
              f'({len(angles)} positions, {REVS_PER_ANGLE} rev each)')
        t0 = time.time()
        try:
            for i, target in enumerate(angles):
                actual = goto(c_port, target)
                samples = grab(s, REVS_PER_ANGLE)
                valid = [(actual, phi, d / 1000.0, q)
                         for q, phi, d in samples if q > 0 and d > 0]
                rows += valid
                if i % 10 == 0 or i == len(angles) - 1:
                    print(f'  {actual:+7.2f} deg  {len(valid):4d} pts   '
                          f'total {len(rows):,}   {time.time() - t0:5.1f} s')
        except KeyboardInterrupt:
            aborted = True
            print(f'\naborted - keeping {len(rows):,} points')

        try:
            park(c_port)
        except (KeyboardInterrupt, RuntimeError) as e:
            print(f'could not return the head: {e or "interrupted"}')
    finally:
        lidar_stop(s)
        s.close()
        c_port.close()

    if not rows:
        print('nothing captured')
        return
    arr = np.array(rows)
    stamp = time.strftime('%Y%m%d_%H%M%S') + ('_partial' if aborted else '')
    np.savetxt(f'scan_{stamp}_raw.csv', arr, delimiter=',',
               header='theta_deg,phi_deg,range_m,quality', comments='')
    print(f'saved scan_{stamp}_raw.csv')
    build_cloud(arr, f'scan_{stamp}.ply')


if __name__ == '__main__':
    if len(sys.argv) == 2:
        rebuild(sys.argv[1])
    elif len(sys.argv) == 4:
        capture(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    else:
        capture(START_DEG, STOP_DEG, STEP_DEG)
