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
        AllCells[r].append([-1,'true','true',[0,0,0,0]])

obstacleID = 0
val = 0
for o in LargeTunnels:
    wall = [2,2,2,2]
    for i in range(4):
        o[0][i] *= 4
    for r in range(int(o[0][1]-o[0][0])):
        for c in range(int(o[0][3]-o[0][2])):
            val = 3
            if(o[1] == 'h'):
                if(r == int(o[0][1]-o[0][0]) - 1):
                    wall = [2,-1,2,2]
                    val = 5
                elif(r == 0):
                    wall = [2,2,-1,2]
                    val = 5
            elif(o[1] == 'v'):
                if(c == int(o[0][3]-o[0][2]) - 1):
                    wall = [2,2,2,-1]
                    val = 5
                elif(c == 0):
                    wall = [-1,2,2,2]
                    val = 5
            Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] = obstacleID
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][3] = wall
    obstacleID += 1

for o in SmallTunnels:
    wall = [1,1,1,1]
    for i in range(4):
        o[0][i] *= 4
    for r in range(int(o[0][1]-o[0][0])):
        for c in range(int(o[0][3]-o[0][2])):
            val = 2
            if(o[1] == 'h'):
                if(r == int(o[0][1]-o[0][0]) - 1):
                    wall = [1,-1,1,1]
                    val = 5
                elif(r == 0):
                    wall = [1,1,-1,1]
                    val = 5
            if(o[1] == 'v'):
                if(c == int(o[0][3]-o[0][2]) - 1):
                    wall = [1,1,1,-1]
                    val = 5
                elif(c == 0):
                    wall = [-1,1,1,1]
                    val = 5
            Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] = obstacleID
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][3] = wall
    obstacleID += 1

for o in Ramps:
    wall = [0,0,0,0]
    for i in range(4):
        o[0][i] *= 4
    for r in range(int(o[0][1]-o[0][0])):
        for c in range(int(o[0][3]-o[0][2])):
            val = 1
            if(o[1] == 'h'):
                if(r == int(o[0][1]-o[0][0]) - 1):
                    wall = [0,-1,0,0]
                    val = 5
                elif(r == 0):
                    wall = [0,0,-1,0]
                    val = 5
            if(o[1] == 'v'):
                if(c == int(o[0][3]-o[0][2]) - 1):
                    wall = [0,0,0,-1]
                    val = 5
                elif(c == 0):
                    wall = [-1,0,0,0]
                    val = 5
                print(r,c,val)
            Obstacles[r+int(o[0][0])+100][c+int(o[0][2])+100] = val
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][0] = obstacleID
            AllCells[r+int(o[0][0])+100][c+int(o[0][2])+100][3] = wall
    obstacleID += 1


H = np.array(Obstacles)  # added some commas and array creation code

fig = plt.figure(figsize=(6, 3.2))

ax = fig.add_subplot(111)
ax.set_title('colorMap')
plt.imshow(H)
ax.set_aspect('equal')
plt.show()

with open('./hare.map', 'w+') as map:
    for r in range(200):
        str = '{\n'
        for c in range(200):
            str += '\t(map_node){'
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
        str += '}'
        if(r != 199):
            str += ','
        map.write(str)
