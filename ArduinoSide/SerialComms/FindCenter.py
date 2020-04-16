import serial
from time import sleep

port = "/dev/ttyUSB0"
# sleep(1)
ser = serial.Serial(port,19200,timeout=3)
sleep(2)
timer = 0.1

while True:
    theta = input("theta:")
    phi = input("phi:")
    command = f'{theta},{phi}\n'
    print("Python sending:", command.encode())
    ser.flush()
    ser.write(command.encode())
    msg = ser.readline()
    print("Message from arduino:",msg)
    sleep(timer)
