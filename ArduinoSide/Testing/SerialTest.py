from serial import Serial
port = "/dev/ttyUSB0"
SFA = Serial(port,9600)
SFA.flushInput()
while True:
    input = SFA.readline()
    inputAsInt = int(input)
    print(inputAsInt*10)
