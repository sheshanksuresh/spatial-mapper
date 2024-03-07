#Sheshank Suresh
#400172679
#Python 3.6.8
#Imported open3d, numpy and math
import open3d as o3d
import math
import numpy as np

pcd = o3d.io.read_point_cloud("FINALXYZ.xyz", format = 'xyz')


lines = [] #Create a list

#Nested for loop to continously connect the first point to the last point and creating multiple lists of lists
for i in range(4): 
    for j in range(256):
        lines.append([j+i*256,j+i*256+256])

#Nested loops to connect each point to the next one until it hits the last point
for i in range(4):
    for j in range(256):
        if j==255: #If statement to connect the last point to 0th point
            lines.append([j+i*256,0+i*256])
        else:
            lines.append([j+i*256,j+1+i*256])

            
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
line_set.lines = o3d.utility.Vector2iVector(lines)

#Show results
o3d.visualization.draw_geometries([line_set])
    
