# Program to established advanced communication between two interfaces (i.e., deliver commands)

import serial 
import time

try:
    arduino = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=5, 
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        writeTimeout=2
    )
    print('Connection to Arduino established')
except serial.SerialException as e:
    print('FAILED TO CONNECT TO ARDUINO {0}'.format(e))

while True:
    try:
        command = input("Enter command (A/B/C): ").strip().upper()
        arduino.write((command + "|").encode())
        data = arduino.readline().decode().strip()
        if data:
            print("Arduino response:", data)
        time.sleep(1)
    except Exception as e:
        print(e)
        arduino.close()