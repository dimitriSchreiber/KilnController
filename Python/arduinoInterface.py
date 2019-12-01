#General use
import time
from datetime import datetime
from datetime import timedelta

import numpy as np

#Microcontroller interfacing
import serial
import re

#Data reading and logging
import json
import csv

#Spline fitting
from scipy import interpolate

#Scheduler
from apscheduler.schedulers.background import BackgroundScheduler

#Plotting
import matplotlib.pyplot as plt

#--------------------------------------
#Interfacing class
#--------------------------------------
class arduinoInterface():
    def __init__(self, dt = 5.0):
        #User defined parameters
        self.dt = dt

        #Kiln state variables
        self.internalTemperature = 0.0
        self.kilnTemperature = 0.0
        self.kilnOn = False
        self.kilnHeartbeat = False
        self.startTime = time.time()

        #AP scheduler for PWM generation
        self.scheduler = BackgroundScheduler()
        self.scheduler.start()

    #def __del__(self):
    #    self.disconectSerial()

    def connectSerial(self, serialDevice):
        self.ser = serial.Serial(serialDevice, 115200) 
        self.turnKilnOff()

    def disconectSerial(self):
        self.turnKilnOff()
        self.scheduler.shutdown(wait=True)
        self.ser.close()

    def turnKilnOn(self):
        self.ser.write(str(1).encode())
        self.readLineFromArduino()

    def turnKilnOff(self):
        self.ser.write(str(0).encode())
        self.readLineFromArduino()

    def readLineFromArduino(self):
        readLine = str(self.ser.readline())
        splitList = re.split(' ', readLine)

        self.kilnTemperature = float(splitList[1])
        self.internalTemperature = float(splitList[2])
        self.kilnOn = int(splitList[3])
        self.kilnHeartbeat = int(splitList[5])

    def setKilnDutyCycle(self, dutyCycle):
        if dutyCycle > 0.0:
            onTime = dutyCycle * self.dt
        else:
            onTime = 0.0

        offTime = self.dt - onTime
        offTimeTime = datetime.now() + timedelta(seconds = onTime)

        if onTime > 0.0:
            self.scheduler.add_job(self.turnKilnOn, 'date', run_date = datetime.now())

        if offTime > 0.0:
            self.scheduler.add_job(self.turnKilnOff, 'date', run_date = offTimeTime)

#--------------------------------------
#Temperature controller class
#--------------------------------------
class TemperatureController():
    def __init__(self, dt, interfaceObject, heatingElementPower = 1500):
        #User defined parameters
        self.dt = dt
        self.interfaceObject = interfaceObject

        #Enable temperature controller
        self.enabled = False

        #Setpoints and measured temperatures at each control cycle
        self.setpoint = 0.0
        self.temperatureSetpoints = []
        self.temperaturesMeasuredKiln = []
        self.temperaturesMeasuredInternal = []
        self.timesExecuted = []


        #AP scheduler for controller execution
        self.scheduler = BackgroundScheduler()
        self.scheduler.start(paused=True)
        self.scheduler.add_job(self.temperatureControl, 'interval', seconds = self.dt)

    #def __del__(self):
    #    self.disableController()
    #    self.scheduler.shutdown()

    def enableController(self):
        self.scheduler.resume()
        #Start time for time keeping subtraction
        self.startTime = time.time()

    def shutdownController(self):
        self.scheduler.pause()
        self.scheduler.shutdown(wait=True)

    def updateSetpoint(self, setpoint):
        self.setpoint = setpoint

    def dumpData(self, dumpFile):
        with open(dumpFile, mode='w') as logging_file:
            logging_writer = csv.writer(logging_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for setpoint, kilnTemp, internalTemp, timeElement in zip(self.temperatureSetpoints, self.temperaturesMeasuredKiln, self.temperaturesMeasuredInternal, self.timesExecuted):
                logging_writer.writerow([setpoint, kilnTemp,internalTemp,timeElement])

    def calculateControlInput(self, setpoint):
        dutyCycle = 1.0 * (setpoint - self.interfaceObject.kilnTemperature)
        return np.clip(dutyCycle, 0.0, 1.0)

    def temperatureControl(self):
        dutyCycle = self.calculateControlInput(self.setpoint)
        self.interfaceObject.setKilnDutyCycle(dutyCycle)

        self.temperatureSetpoints.append(self.setpoint)
        self.temperaturesMeasuredKiln.append(self.interfaceObject.kilnTemperature)
        self.temperaturesMeasuredInternal.append(self.interfaceObject.internalTemperature)
        self.timesExecuted.append(time.time()-self.startTime)

        print('Time: {}, Setpoint Temperature: {}, Kiln Temperature: {}, Dutycycle: {}'.format(
            (time.time() - self.startTime)/3600, self.setpoint, self.interfaceObject.kilnTemperature, dutyCycle))

#--------------------------------------
#Temperature profile class
#--------------------------------------
class TemperatureProfile():
    def __init__(self, dt, controllerObject, temperatureFile):
        self.temperatureFile = temperatureFile
        self.dt = dt

        self.controllerObject = controllerObject

        #Read in temperature profile
        self.temperatureSetpoints = []
        self.timeSetpoints = []
        self.readTemperatureProfile()

        #Interpolated temperature profile based on dt
        self.temperatureSetpointsInterpolated = []
        self.timeSetpointsInterpolated = []
        self.fitSpline()
        self.interpolateProfile()

        #index in profile
        self.index = 0
        self.done = False

        #profile executing scheduler
        self.scheduler = BackgroundScheduler()
        self.scheduler.start(paused=False)
        self.scheduler.add_job(self.execute, 'interval', seconds = self.dt)

    def shutdown(self):
        self.scheduler.pause()
        self.scheduler.shutdown(wait=True)

    def readTemperatureProfile(self):
        print('*********************************************************************')
        print('*********************************************************************')
        print('Reading temperature profile from JSON: {}'.format(self.temperatureFile))
        print('*********************************************************************')

        with open(self.temperatureFile) as json_file:
            data = json.load(json_file)
            for setpoint in data['setpoints']:
                temperature = setpoint['temperature']
                setpointTime = setpoint['time']
                print('Temperature: {}'.format(temperature))
                print('Time: {}'.format(setpointTime))
                self.temperatureSetpoints.append(float(temperature))
                self.timeSetpoints.append(float(setpointTime))
            self.temperatureSetpoints = (np.array(self.temperatureSetpoints) - 32) * 5.0 / 9.0
            self.timeSetpoints = np.array(self.timeSetpoints)

        self.timeCumSum = np.cumsum(self.timeSetpoints)
        print('Cumulative hours for profile: {}'.format(self.timeCumSum))
        print('*********************************************************************')


    def fitSpline(self):
        self.f = interpolate.interp1d(self.timeCumSum*3600, self.temperatureSetpoints,
         kind='linear', fill_value="extrapolate")

    def interpolateProfile(self):
        self.timeSetpointsInterpolated = np.arange(0, self.timeCumSum[-1]*3600, self.dt)
        print('Interpolated time (seconds): {}'.format(self.timeSetpointsInterpolated))
        self.temperatureSetpointsInterpolated = self.f(self.timeSetpointsInterpolated)
        print('Interpolated temperature (Celsius): {}'.format(self.temperatureSetpointsInterpolated))
        print('Plotting...')
        plt.figure()
        plt.plot(self.timeSetpointsInterpolated/3600, self.temperatureSetpointsInterpolated)
        plt.xlabel('Time (hours)')
        plt.ylabel('Temperature (Celsius)')
        plt.title('Interpolated Profile')
        plt.show()
        print('*********************************************************************')
        print('*********************************************************************')

    def execute(self):
        if self.index < len(self.timeSetpointsInterpolated):
            self.controllerObject.updateSetpoint(self.temperatureSetpointsInterpolated[self.index])
        else:
            self.done = True
        self.index += 1




if __name__ == '__main__': 

    dt = 1.0

    interface = arduinoInterface(dt = dt)
    interface.connectSerial("/dev/ttyACM0")
    controller = TemperatureController(dt = dt, interfaceObject = interface)
    profile = TemperatureProfile(dt = dt, controllerObject = controller, temperatureFile = 'cone06Glaze.txt')
    controller.enableController()

    start=time.time()

    while not profile.done:
        time.sleep(1)

    controller.dumpData('test.csv')

    profile.shutdown()
    controller.shutdownController()
    interface.disconectSerial()
    exit()