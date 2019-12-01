import serial
import time
import re
import numpy as np

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Setup serial communication
ser = serial.Serial("/dev/ttyACM0", 115200)   # open serial port that Arduino is using
print(ser)                           # print serial config

#Record code start time
startTime = time.time()

#Setup temperature profile

def readLineFromArduino():
	readLine = str(ser.readline());
	splitList = re.split(' ', readLine);

	internalTemperature = float(splitList[2])
	kilnTemperature = float(splitList[1])
	kilnOn = int(splitList[3])
	kilnHeartbeat = int(splitList[5])

	return internalTemperature, kilnTemperature, kilnOn, kilnHeartbeat

def animate(i, xs, ys):

	ser.write(str(1).encode())

	internalTemperature, kilnTemperature, kilnOn, kilnHeartbeat = readLineFromArduino()
	print(kilnTemperature)



	# Read temperature (Celsius) from TMP102
	temp_c = round(kilnTemperature, 2)

	# Add x and y to lists
	#xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
	xs.append(time.time()-startTime)
	ys.append(temp_c)

	# Limit x and y lists to 20 items
	xs = xs[-2000:]
	ys = ys[-2000:]

	# Draw x and y lists
	ax.clear()
	ax.plot(xs, ys)

	# Format plot
	plt.xticks(rotation=45, ha='right')
	plt.subplots_adjust(bottom=0.30)
	plt.title('TMP102 Temperature over Time')
	plt.ylabel('Temperature (deg C)')

	#ser.write(str(0).encode())
	#time.sleep(1)
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=10000)
plt.show()


while(time.time() - startTime < 30):
	print ('runnin')

# Reminder to close the connection when finished
if(ser.isOpen()):
   print("Serial connection is still open.")

ser.close()