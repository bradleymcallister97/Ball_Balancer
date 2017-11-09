import Serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

ser = serial.Serial('COM9', 9600, timeout =1)

while 1:
    try:
        print ser.readline()
        time.sleep(1)
    except ser.SerialTimeoutException:
        print('Data could not be read \r\n')
        time.sleep(1)

    data = ser.readline()
    # PARSE SERIAL DATA FROM HERE