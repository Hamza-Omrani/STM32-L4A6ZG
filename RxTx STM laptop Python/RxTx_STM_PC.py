import serial
import time
from datetime import datetime

d = 0;
ser = serial.Serial(
    port='COM3',\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("Device Connected Successfully via : " + ser.portstr)

while True:
    Tx = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') # convert datetime to str 
    Tx = str.encode(Tx)  # convert str to bytes 
    ser.write(Tx) # send bytes via usb 
    Rx = ser.readline()
    if Rx.strip():
        d+=1
    print(Rx.decode())
    print("Pong %d"%d)
    time.sleep(1)