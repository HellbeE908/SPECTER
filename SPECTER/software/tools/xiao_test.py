"""
Test the XIAO nod-axis controller from the Raspberry Pi.
Sends a sequence of moves and prints each reply.
"""
import serial
import time

# Run:  ls /dev/serial/by-id/   and paste the XIAO's entry here
PORT = '/dev/serial/by-id/PASTE-XIAO-NAME-HERE'

x = serial.Serial(PORT, 115200, timeout=10)
time.sleep(2)                 # the XIAO may reset when the port opens
x.reset_input_buffer()


def cmd(c):
    x.write((c + '\n').encode())
    reply = x.readline().decode(errors='replace').strip()
    print(f'{c:>7}  ->  {reply or "(no reply)"}')
    return reply


cmd('E')
cmd('?')
for angle in [10, 20, 30, 0, -10, -20, -30, 0]:
    t = time.time()
    cmd(f'G{angle}')
    print(f'          took {time.time() - t:.2f} s')
cmd('?')
x.close()
