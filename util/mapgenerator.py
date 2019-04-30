#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt


# left   -> right
# bottom -> top
# [x1,x2,y1,y2],type
#
# Obstacles entered by type, from top to bottom


#height = 2
LargeTunnels = [
    ([-10,-6,6,8],'v'),#short large green
    ([1,5,3,5],'v'),#short large green
    ([-1,1,-10,-2],'h'),#large blue
    ([2,10,-1,1],'v'),#large green
    ([-7,-5,-6,-2],'h'),#large blue
]
#height = 1
SmallTunnels = [
    ([-6,-4,8,12],'h'),#small red
    ([1,3,5,9],'h'),#small red
    ([5,9,3,5],'v'),#short small green
    ([-5,-1,-12,-10],'v'),#short wide green
    ([-7,-5,-2,2],'h'),#small red
    ([-7,-5,-10,-6],'h'),#small red
]
Ramps = [
    ([3.5,8.5,-3,-1],'v'),#narrow ramp
    ([-5,-1,-7.5,-2.5],'h')#wide ramp
]

#other is
#large cube - small cube
#large cube = 2x2x2
#small cube = 1x1x1
large1 = [9,11,9,11]
small1 = [10,11,10,11]
large2 = [-7,-9,-7,-9]
small2 = [-8,-9,-7,-8]
large3 = [-12,-14,7,9]
small3 = [-12,-13,8,9]

Obstacles_raw = [LargeTunnels,SmallTunnels,Ramps]
AllCells = []
Obstacles = []
for r in range(200):
    Obstacles.append([])
    AllCells.append([])
    for c in range(200):
        Obstacles[r].append(0)
        AllCells[r].append([0,'true','true',[0,0,0,0]])

obstacleID = 0
val = 0
for o in LargeTunnels:
    for i in range(4):
        o[0][i] *= 4
    xdist = int(o[0][1]-o[0][0])
    if(o[1] == 'h'):
        xdist += 2
    ydist = int(o[0][3]-o[0][2])
    if(o[1] == 'v'):
        ydist += 2
    for r in range(xdist):
        for c in range(ydist):
            wall = [3,3,3,3]
            traversable = 'true'
            if(AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] == -1):
                continue
            val = 3
            if(o[1] == 'h'):
                if(r >= int(o[0][1]-o[0][0])):
                    traversable = 'false'
                    wall = [3,-1,3,3]
                    val = -1
                elif(r <= 1):
                    traversable = 'false'
                    wall = [3,3,-1,3]
                    val = -1
            elif(o[1] == 'v'):
                if(c >= int(o[0][3]-o[0][2])):
                    traversable = 'false'
                    wall = [3,3,3,-1]
                    val = -1
                elif(c <= 1):
                    traversable = 'false'
                    wall = [-1,3,3,3]
                    val = -1
            Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][2] = traversable
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][3] = wall

for o in SmallTunnels:
    for i in range(4):
        o[0][i] *= 4
    xdist = int(o[0][1]-o[0][0])
    if(o[1] == 'h'):
        xdist += 2
    ydist = int(o[0][3]-o[0][2])
    if(o[1] == 'v'):
        ydist += 2
    for r in range(xdist):
        for c in range(ydist):
            wall = [2,2,2,2]
            traversable = 'true'
            if(Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] == -1):
                continue
            val = 2
            if(o[1] == 'h'):
                if(r >= int(o[0][1]-o[0][0])):
                    traversable = 'false'
                    wall = [2,-1,2,2]
                    val = -1
                elif(r <= 1):
                    traversable = 'false'
                    wall = [2,2,-1,2]
                    val = -1
            if(o[1] == 'v'):
                if(c >= int(o[0][3]-o[0][2])):
                    traversable = 'false'
                    wall = [2,2,2,-1]
                    val = -1
                elif(c <= 1):
                    traversable = 'false'
                    wall = [-1,2,2,2]
                    val = -1
            Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][2] = traversable
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][3] = wall

for o in Ramps:
    for i in range(4):
        o[0][i] *= 4
    xdist = int(o[0][1]-o[0][0])
    if(o[1] == 'h'):
        xdist += 2
    ydist = int(o[0][3]-o[0][2])
    if(o[1] == 'v'):
        ydist += 2
    for r in range(xdist):
        for c in range(ydist):
            wall = [1,1,1,1]
            traversable = 'true'
            if(AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] == -1):
                continue
            val = 1
            if(o[1] == 'h'):
                if(r >= int(o[0][1]-o[0][0])):
                    traversable = 'false'
                    wall = [1,-1,1,1]
                    val = -1
                elif(r <= 1):
                    traversable = 'false'
                    wall = [1,1,-1,1]
                    val = -1
            if(o[1] == 'v'):
                if(c >= int(o[0][3]-o[0][2])):
                    traversable = 'false'
                    wall = [1,1,1,-1]
                    val = -1
                elif(c <= 1):
                    traversable = 'false'
                    wall = [-1,1,1,1]
                    val = -1
            Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][2] = traversable
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][3] = wall


H = np.array(Obstacles)  # added some commas and array creation code

fig = plt.figure(figsize=(6, 3.2))

ax = fig.add_subplot(111)
ax.set_title('colorMap')
plt.imshow(H)
ax.set_aspect('equal')
plt.show()

with open('./HareMap.cpp', 'w+') as map:
    map.write('#include "HareMap.h"\n')
    map.write('hare::map_node hare::fullMap[MAP_X][MAP_Y] {\n')
    for r in range(200):
        str = '\t{\n'
        for c in range(200):
            str += '\t\t(map_node){'
            str += "%s" % AllCells[r][c][0]
            str +=','
            str += "%s" % AllCells[r][c][1]
            str +=','
            str += "%s" % AllCells[r][c][2]
            str +=',(int4){'
            str+= "%s" % AllCells[r][c][3][0]
            str +=','
            str += "%s" % AllCells[r][c][3][1]
            str +=','
            str += "%s" % AllCells[r][c][3][2]
            str +=','
            str += "%s" % AllCells[r][c][3][3]
            str += '}}'
            if(c != 199):
                str += ','
            str += '\n'
        str += '\t}'
        if(r != 199):
            str += ','
        str += '\n'
        map.write(str)
    map.write('};')
