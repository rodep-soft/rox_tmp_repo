import struct
import time

import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


def parse_motor_feedback(data: bytes):
    if len(data) != 10:
        raise ValueError("データ長は10バイトじゃないにゃ！")

    ID, mode, torque_hi, torque_lo, vel_hi, vel_lo, pos_hi, pos_lo, error, crc = (
        struct.unpack("10B", data)
    )
    velocity = (vel_hi << 8) | vel_lo
    return ID, velocity


try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
    print("Reading data. Press Ctrl+C to stop.")

    buffer = b""

    while True:
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)

            # 10バイトずつパケットを処理
            while len(buffer) >= 10:
                packet = buffer[:10]
                buffer = buffer[10:]
                try:
                    motor_id, velocity = parse_motor_feedback(packet)
                    print(f"Motor ID: {motor_id}, Velocity: {velocity}")
                except Exception as e:
                    print(f"パケット解析エラー: {e}, data: {packet.hex().upper()}")
        time.sleep(0.01)

except serial.SerialException as e:
    print(f"Error opening or reading serial port: {e}")
except KeyboardInterrupt:
    print("\nStopping serial reader.")
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
