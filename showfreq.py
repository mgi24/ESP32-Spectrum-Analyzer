import serial
import numpy as np
import cv2

# Configure the serial connection
com_port = 'COM3'  # Replace with your COM port
baud_rate = 115200  # Communication speed

try:
    # Open the serial connection
    with serial.Serial(com_port, baud_rate, timeout=1) as ser:
        print(f"Connected to {com_port} at {baud_rate} baud.")
        print("Press Ctrl+C to exit.\n")

        while True:
            # Read data from the ESP32
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                # Create a black image with dimensions 512x1024
                image = np.zeros((1024, 12, 3), dtype=np.uint8)

                # Parse the data received from ESP32
                values = data.split(" ")
                for i, value in enumerate(values):
                    try:
                        print(f"Value {i}: {value}")
                        x = i  # X coordinate is the index
                        y = int(float(value)/1)  # Y coordinate is the value
                        cv2.line(image, (x, 0), (x, y), (255, 255, 255), 1)  # Draw a white line
                    except ValueError:
                        print(f"Invalid value: {value}")

                # Display the image in a resizable window
                cv2.namedWindow("ESP32 Data Visualization", cv2.WINDOW_NORMAL)
                cv2.imshow("ESP32 Data Visualization", image)
                cv2.waitKey(1)

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("\nExiting...")