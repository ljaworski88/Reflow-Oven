import serial
import numpy
import matplotlib.pyplot as plt
from drawnow import *

setpoints = []
avgTemp = []
PIDoutput = []
tempData = serial.Serial('COM3', 115200)
plt.ion()
tempBuffer = 0

def makeFig():
    plt.ylim(20, 250)
    plt.xlim(0, 360)
    plt.title('Reflow Controller Temp and Setpoint Data')
    plt.grid(True)
    plt.ylabel('Temp C')
    plt.plot(setpoints, 'ro-', label='Setpoints')
    plt.plot(avgTemp, 'b^-', label='Temperature')
    plt.legend(loc='upper left')
    plt2 = plt.twinx()
    plt.ylim(0,100)
    plt2.plot(PIDoutput, 'go-', label = 'PID Output')
    plt2.set_ylabel('PID')
    plt2.ticklabel_format(useOffset = False)
	
    

while True:
    while (tempData.inWaiting()==0):
        pass
    dataString = tempData.readline()
    dataArray = dataString.split(',')
    tempPID = float(dataArray[0])
    tempSetpoint = float(dataArray[1])
    tempTemp = float(dataArray[2])
    PIDoutput.append(tempPID)
    setpoints.append(tempSetpoint)
    avgTemp.append(tempTemp)
    drawnow(makeFig)
    plt.pause(0.000001)
    tempBuffer += 1
    if(tempBuffer > 360):
         PIDoutput.pop(0)
         setpoints.pop(0)
         avgTemp.pop(0)
	
