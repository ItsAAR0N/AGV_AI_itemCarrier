# Program to establish communication between the Jetson Nano and Arduino

import serial 
import time

# arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1)
try:
    arduino = serial.Serial(
        port = '/dev/ttyUSB0',
        baudrate = 115200,
        bytesize = serial.EIGHTBITS,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        timeout = 5, 
        xonxoff = False,
        rtscts = False,
        dsrdtr = False,
        writeTimeout = 2
    )
    print('Connection to Arduino established')
except serial.SerialException as e:
    print('FAILED TO CONNECT TO ARDUINO {0}'.format(e))

while True:
    try:
        arduino.write("Command from Jetson|".encode())
        data = arduino.readline()
        if data:
            print(data) # Print received from Arduino to console
        time.sleep(1)
    except Exception as e:
        print(e)
        arduino.close()