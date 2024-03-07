import serial
import math

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

i = 0
checker = '-, '

s = serial.Serial("COM4", 115200)
print("Opening: "+ s.name)

angle = []
ranges = []
distances = []
yValues = []
zValues = []

try:
    while(i<500):
        line = s.readline().strip()
        values = line.decode()
        print(values)
        dataFile = open('dataFile2.txt', 'a')
        values = values.partition(checker)[2]
        dataFile.write(values + "\n")
        i+=1
        dataFile.close()
except KeyboardInterrupt:
    print("Program Stopped")


print("Closing: " + s.name)
s.close()