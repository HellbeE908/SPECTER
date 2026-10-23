"""
RPLIDAR C1 diagnostic for Raspberry Pi 5.

Talks to the sensor directly (no rplidar library), starts the motor
explicitly, and logs the Pi's 5 V supply before and after the motor
starts so we can see whether the voltage dips.

Unplug and replug the C1 before each run while debugging.
"""
import serial
import statistics
import subprocess
import time

# ---------------------------------------------------------------- settings
PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_bad54130f45df01185735a401045c30f-if00-port0'
BAUD = 460800
SPINUP_S = 3.0        # seconds to let the motor reach speed
WARMUP_REVS = 5       # revolutions discarded while the motor settles
SAMPLE = 30           # good revolutions used for the statistics
MIN_VALID = 50        # revolutions with fewer valid points are discarded

# ---------------------------------------------------------------- protocol
CMD_STOP = b'\xA5\x25'
CMD_SCAN = b'\xA5\x20'
MOTOR_ON = b'\xA5\xF0\x02\x94\x02\xC1'    # motor PWM 660
MOTOR_OFF = b'\xA5\xF0\x02\x00\x00\x57'   # motor PWM 0
SCAN_HEADER = b'\xA5\x5A\x05\x00\x00\x40\x81'


def supply_voltage():
    """Pi 5 input voltage in volts, or None if it can't be read."""
    try:
        out = subprocess.run(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'],
                             capture_output=True, text=True, timeout=2).stdout
        return float(out.split('=')[1].strip().rstrip('V'))
    except Exception:
        return None


def fmt_v(v):
    return f'{v:.3f} V' if v is not None else 'unavailable'


def start_scan(s):
    s.dtr = False                       # motor enable line on the adapter
    s.write(CMD_STOP)                   # clear any scan left running
    time.sleep(0.5)
    s.reset_input_buffer()

    v_before = supply_voltage()
    s.write(MOTOR_ON)
    print(f'motor on - waiting {SPINUP_S:.0f} s for it to reach speed...')
    print('  >>> WATCH THE LIDAR HEAD NOW <<<')
    time.sleep(SPINUP_S)
    v_after = supply_voltage()

    print(f'supply before motor : {fmt_v(v_before)}')
    print(f'supply motor running: {fmt_v(v_after)}')

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
        s.dtr = True                    # motor line off
    except Exception:
        pass


def revolutions(s, progress):
    """Yield one revolution at a time as a list of (quality, angle_deg, dist_mm)."""
    rev = []
    buf = bytearray(s.read(5))
    while True:
        if len(buf) < 5:
            raise RuntimeError('sensor stopped sending data')

        start = buf[0] & 1
        start_inv = (buf[0] >> 1) & 1
        check = buf[1] & 1
        if start == start_inv or check != 1:      # out of sync: slide one byte
            buf = buf[1:] + s.read(1)
            continue

        quality = buf[0] >> 2
        angle = ((buf[1] >> 1) | (buf[2] << 7)) / 64.0
        dist = (buf[3] | (buf[4] << 8)) / 4.0
        progress['nodes'] += 1

        if start and rev:
            progress['revs'] += 1
            yield rev
            rev = []
        rev.append((quality, angle, dist))
        buf = bytearray(s.read(5))


def report(counts, quals, dists):
    if not counts:
        print('\nno usable revolutions collected')
        return
    mean = statistics.mean(counts)
    print(f'\n--- {len(counts)} good revolutions ---')
    print(f'points/rev : {mean:.0f}  (min {min(counts)}, max {max(counts)}, '
          f'sd {statistics.pstdev(counts):.0f})')
    print(f'range      : {min(dists):.2f} - {max(dists):.2f} m, '
          f'median {statistics.median(dists):.2f} m')
    print(f'quality    : {min(quals)} - {max(quals)}, '
          f'{len(set(quals))} distinct values')
    print(f'\nprojected at 220 nod steps : {mean * 220:,.0f} points')


def main():
    s = serial.Serial(PORT, BAUD, timeout=5)
    progress = {'nodes': 0, 'revs': 0}
    t0 = time.time()
    try:
        start_scan(s)
        print('scan started - collecting...\n')

        counts, quals, dists = [], [], []
        for i, rev in enumerate(revolutions(s, progress)):
            if i < 10:
                valid_now = sum(1 for m in rev if m[0] > 0 and m[2] > 0)
                print(f'  rev {i:2d}: {len(rev):3d} samples, {valid_now:3d} valid')
            if i < WARMUP_REVS:
                continue
            valid = [m for m in rev if m[0] > 0 and m[2] > 0]
            if len(valid) < MIN_VALID:
                continue
            counts.append(len(valid))
            quals += [m[0] for m in valid]
            dists += [m[2] / 1000 for m in valid]
            if len(counts) >= SAMPLE:
                break

        report(counts, quals, dists)
        print(f'supply at end: {fmt_v(supply_voltage())}')

    except RuntimeError as e:
        print(f'\nFAILED: {e}')
        print(f'  measurements received : {progress["nodes"]}')
        print(f'  full revolutions      : {progress["revs"]}')
        print(f'  time since start      : {time.time() - t0:.1f} s')
        print(f'  supply now            : {fmt_v(supply_voltage())}')

    finally:
        stop_scan(s)
        s.close()
        print('stopped cleanly')


if __name__ == '__main__':
    main()
