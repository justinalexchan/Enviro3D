#Justin Chan
#Requires pyserial
from serial import Serial
import serial
import math


#Convert Distance to coordinate system
def distance2Coordinate(arr, distance, angle, x):
    arr.append(x)
    arr.append(int(math.sin(angle * math.pi/180.0) * float(distance)))
    arr.append(int(math.cos(angle * math.pi/180.0) * float(distance)))

#Print coordinates
def printCoord(arr):
    for i in range(len(arr)):
        print(arr[i], end = '')
        if(i%3 == 2 and i != 0):
            print()
        else:
            print(end = ' ')

def writeToFile(arr):
    f = open("coordinates3D.xyz", "a")
    for i in range(len(arr)):
        f.write(str(arr[i]))
        if(i%3 == 2 and i != 0):
            f.write("\n")
        else:
            f.write(" ")
    f.close()
#Var
dataPoints = 0          #This counts the number of distance data point that are sent through 
coordinates = []
preNum = ''
xValue = 0
angle = 0

#Open port 
s = serial.Serial("COM4", 115200)

print("Opening: " + s.name)

s.write(b'1')           #This program will send a '1' or 0x31 

#Open/Creating a Blank xyz file
f = open("coordinates3D.xyz", "w")
f.close()

while 1:
    while dataPoints != 64:     #Each point will be taken every 8 steps for a total of 64 points
        x = s.read()        # read one byte
        c = x.decode()      # convert byte type to str

        if c == "\n":
            #print(preNum)
            distance2Coordinate(coordinates, preNum, angle, xValue)
            angle += 5.625 # 8/512 * 360
            dataPoints += 1
            preNum = ''
        else:
            preNum += c
    dataPoints = 0
    xValue += 100
    #Append to file
    writeToFile(coordinates)
    #Clear coodinates array
    coordinates = []


#printCoord(coordinates)
print("Closing: " + s.name)
s.close()
