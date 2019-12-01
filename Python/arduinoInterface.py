import serial
import time
import re
import numpy as np

import json
import csv

class arduinoInterface():
    def __init__(self, fig, **kw):
        self.internalTemperature = 0.0
        self.kilnTemperature = 0.0
        self.kilnOn = False
        self.kilnHeartbeat = False

        self.startTime = time.time()

        self.temperatureSetpoints = []
        self.timeSetpoints = []

        self.temperatureSetpointsInterpolated = []
        self.timeSetpointsInterpolated = []
        self.temperaturesMeasuredKiln = []
        self.temperaturesMeasuredInternal = []
        self.timesExecuted = []

    def connectSerial(self, serialDevice):
        self.ser = serial.Serial(serialDevice, 115200) 

    def disconectSerial(self):
        self.ser.close()

    def readTemperatureProfile(self, fileDirectory):
        pass

    def turnKilnOn(self, on):
        if on:
            self.ser.write(str(1).encode())
        else:
            self.ser.write(str(0).encode())
        timesExecuted.append(time.time()-self.startTime)

    def readLineFromArduino(self):
        readLine = str(self.ser.readline())
        splitList = re.split(' ', readLine)

        self.internalTemperature = float(splitList[2])
        self.kilnTemperature = float(splitList[1])
        self.kilnOn = int(splitList[3])
        self.kilnHeartbeat = int(splitList[5])

        self.temperaturesMeasuredKiln.append(self.kilnTemperature)
        self.temperaturesMeasuredInternal.append(self.internalTemperature)

    def dumpData(self, dumpFile):
        with open(dumpFile, mode='w') as logging_file:
            logging_writer = csv.writer(logging_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for kilnTemp, internalTemp, timeElement in zip(self.temperaturesMeasuredKiln, self.temperaturesMeasuredInternal, self.timesExecuted):
                logging_writer.writerow([kilnTemp,internalTemp,timeElement])

if __name__ == '__main__': 

    interface = arduinoInterface()
    interface.connectSerial("/dev/ttyACM0")

    start=time.time()
    while(time.time() - start < 20):
        interface.turnKilnOn(True)
        interface.readLineFromArduino()
        print('Kiln Temperature: {}'.format(interface.kilnTemperature))

    interface.dumpData('test.csv')

    interface.disconectSerial()






