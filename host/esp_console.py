"""Terminal for the ESP32's serial console. Opens the port without toggling DTR/RTS (which
would reset the ESP32-S3 or put it in download mode) and starts the input prompts.

  python3 host/esp_console.py              # /dev/ttyACM0
  python3 host/esp_console.py --port /dev/ttyACM1

Type the values when asked; Enter keeps the value in brackets. Ctrl-C quits.
"""
import argparse
import sys
import threading
import time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/ttyACM0")
a = ap.parse_args()

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = a.port, 115200, 0.1
ser.dtr = ser.rts = False
ser.open()


def reader():
    while True:
        data = ser.read(256)
        if data:
            sys.stdout.write(data.decode(errors="replace"))
            sys.stdout.flush()


threading.Thread(target=reader, daemon=True).start()
time.sleep(0.3)
ser.reset_input_buffer()
ser.write(b"\n")                          # an empty line starts the prompts
try:
    for line in sys.stdin:
        ser.write(line.rstrip("\r\n").encode() + b"\n")
except KeyboardInterrupt:
    pass
print()
