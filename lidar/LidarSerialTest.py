import serial

port = serial.Serial("/dev/ttyS0",baudrate=115200)

while True:

    rcv = port.read()

    print str(rcv)
