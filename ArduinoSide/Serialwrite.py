import serial
from time import sleep
port = "/dev/ttyUSB0"
sleep(1)
ser = serial.Serial(port,9600)
sleep(2)
for iter in range(0,10):
    ser.write(b'17001300')
    sleep(2)
    ser.write(b'15001500')
    sleep(2)
    ser.write(b'13001700')
    sleep(2)
# while True:
#     print("Sending")
#     sleep(10)
