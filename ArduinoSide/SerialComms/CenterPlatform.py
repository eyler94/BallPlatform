import serial
from time import sleep

port = "/dev/ttyUSB0"
# sleep(1)
ser = serial.Serial(port,19200,timeout=3)
sleep(2)
theta_div = 1575
phi_div = 1415
div = str(theta_div) + "," + str(phi_div) + "\n"

print("Python sending:",div.encode())
ser.flush()
ser.write(div.encode())
msg = ser.readline()
print("Message from arduino:",msg)
