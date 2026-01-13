import serial

def test_serial_connection():
    try:
        # Open serial connection
        ser = serial.Serial('/dev/serial0', baudrate=921600, timeout=1)
        print("Serial connection opened successfully.")

        # Send test data
        test_message = "Hello Pixhawk"
        ser.write(test_message.encode())
        print(f"Sent: {test_message}")

        # Read response
        response = ser.read(100)  # Read up to 100 bytes
        print(f"Received: {response}")

        # Close connection
        ser.close()
        print("Serial connection closed.")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_serial_connection()