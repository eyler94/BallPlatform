from serial import Serial
port = "/dev/ttyUSB0"
ser = Serial(port,9600)
ser.write(b'500')
