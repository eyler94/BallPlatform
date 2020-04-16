import serial
from time import sleep

port = "/dev/ttyUSB0"
# sleep(1)
ser = serial.Serial(port,19200,timeout=3)
sleep(2)
# theta = 1500
# phi = 1500
# theta_div = 1600
# phi_div = 1300
# reg = str(theta) + "," + str(phi) + "\n"
# div = str(theta_div) + "," + str(phi_div) + "\n"

timer = 0.1

servos = ['phi', 'theta']
for servo in servos:
    theta = 1500
    phi = 1500
    for change in range(-300,300,10):
        if servo == 'phi':
            phi=1500+change
        else: # servo == 'theta'
            theta=1500+change
        command = f'{theta},{phi}\n'
        print("Python sending:", command.encode())
        ser.flush()
        ser.write(command.encode())
        msg = ser.readline()
        print("Message from arduino:",msg)
        sleep(timer)
command = f'{1500},{1500}\n'
print("Python sending:", command.encode())
ser.flush()
ser.write(command.encode())
msg = ser.readline()
print("Message from arduino:",msg)
sleep(timer)
