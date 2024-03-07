import math

angle = []
ranges = []
distances = []
xValues = []
yValues = []
zValues = []

with open("dataFile3.txt", "r") as filestream:
    with open("Output5.txt", "w") as filestreamtwo:
        for line in filestream:
            currentline = line.split(",")
            print(currentline)
            if len(currentline)>1:
                filestreamtwo.write(currentline[0]+currentline[1]+currentline[2]+currentline[3]+'\n')

with open("Output5.txt", "r") as filestream:
    for line in filestream:
        chkLine = line.split(" ")
        print(chkLine)
        xValues.append(chkLine[0])
        angle.append(chkLine[1])
        ranges.append(chkLine[2])
        distances.append(chkLine[3].strip())

xValues = [float(i) for i in xValues] 
angle = [float(i) for i in angle]
ranges = [float(i) for i in ranges]
distances = [float(i) for i in distances]


for i in range(0,len(angle)):
    yCoord = distances[i]*math.cos(math.radians(angle[i]))
    yValues.append(yCoord)
    zCoord = distances[i]*math.sin(math.radians(angle[i]))
    zValues.append(zCoord)


for i in range(0,len(angle)):
    finalOut = open('FINALXYZ.txt', 'a')
    finalOut.write(str(1000*xValues[i]) +' '+ str(yValues[i])+' '+str(zValues[i])+'\n')
    finalOut.close()

for i in range(0,len(angle)):
    finalOut = open('FINALXYZ.xyz', 'a')
    finalOut.write(str(1000*xValues[i]) +' '+ str(yValues[i])+' '+str(zValues[i])+'\n')
    finalOut.close()

print(angle)
print(ranges)
print(distances)
print(xValues)
print(yValues)
print(zValues)
