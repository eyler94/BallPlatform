import serial
from time import sleep

port = "/dev/ttyUSB0"
# sleep(1)
ser = serial.Serial(port,19200,timeout=3)
sleep(2)
theta = 1500
phi = 1500
theta_div = 1600
phi_div = 1300
reg = str(theta) + "," + str(phi) + "\n"
div = str(theta_div) + "," + str(phi_div) + "\n"

timer = 0.003125

for iter in range(0,100):
    # print("div",div,div.encode())
    print("Python sending:",div.encode())
    ser.flush()
    ser.write(div.encode())
    # sleep(timer)
    msg = ser.readline()
    print("Message from arduino:",msg)
    sleep(timer)

    print("Python sending:",reg.encode())
    ser.flush()
    ser.write(reg.encode())
    # sleep(timer)
    msg = ser.readline()
    print("Message from arduino:",msg)
    sleep(timer)
