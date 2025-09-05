import serial
import time

# Initialize serial once when the module is imported
# Adjust the port and baudrate according to your hardware
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
except serial.SerialException as e:
    ser = None
    print(f"[sender] Serial connection failed: {e}")

def send_command(direction, n_steps):
    if ser is None or not ser.is_open:
        print("[sender] Serial not connected.")
        return

    # Format the message however your Arduino expects it
    # Example format: "D:1,S:100\n" (D = direction, S = steps)
    msg = f"D:{direction},S:{n_steps}\n"
    try:
        ser.write(msg.encode('utf-8'))
        print(f"[sender] Sent: {msg.strip()}")
    except serial.SerialException as e:
        print(f"[sender] Failed to send: {e}")

