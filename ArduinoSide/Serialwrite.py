from serial import Serial
ser = Serial(port,9600)
ser.write(b'500')
