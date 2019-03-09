from serial import Serial
port = "/dev/tcyUSB0"
ser = Serial(port,9600)
ser.write(b'500')
