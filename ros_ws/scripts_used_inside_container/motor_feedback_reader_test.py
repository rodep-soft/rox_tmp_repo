import time

import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)  # Capital "S" in Serial
    print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
    print("Reading data. Press Ctrl+C to stop.")

    while True:
        if ser.in_waiting > 0:
            raw_data = ser.read(ser.in_waiting)  # Read all available bytes
            print(f"Received ({len(raw_data)} bytes): {raw_data.hex().upper()}")
        time.sleep(0.01)

except serial.SerialException as e:
    print(f"Error opening or reading serial port: {e}")
except KeyboardInterrupt:
    print("\nStopping serial reader.")
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
