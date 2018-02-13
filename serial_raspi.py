import serial
import time
import sys

#location of serial can be obtained using ls /dev/tty*
ser = serial.Serial('/dev/ttyACM1', 9600)

time.sleep(2)
while True:
       ser.write("hello")
       time.sleep(0.1)
       ser.write("123")
       time.sleep(0.1)
       ser.write("world")
       time.sleep(0.1)
       ser.write("123456789")
       time.sleep(0.1)
       
