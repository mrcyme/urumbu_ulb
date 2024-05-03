import serial
import time

# Setup the serial connections
serial_port0 = '/dev/rfcomm0'
serial_port1 = '/dev/rfcomm1'
baud_rate = 115200 # Adjust as necessary

try:
    ser0 = serial.Serial(serial_port0, baud_rate, timeout=1)
    ser0.reset_input_buffer()
    #ser1 = serial.Serial(serial_port1, baud_rate, timeout=1)
    i = 0
    while i < 200:
        ser0.write(b'f')  # Write the character 'f' to the first serial port
        #ser1.write(b'f')  # Write the character 'f' to the second serial port
        print("Sent 'f' to both motors")
        time.sleep(0.003)  # Short delay to limit the command rate
        i += 1
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
finally:
    ser0.close()  # Ensure the first serial port is closed properly
    #ser1.close()  # Ensure the second serial port is closed properly

