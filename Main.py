import Importing as imp
from graphics import *
import keyboard
import time
import sys

height = 800
width = 1000
anchorpoint = Point(width / 2, height / 2)

win = GraphWin("map_display_window", width, height)
image = Image(anchorpoint, height, width)

'''
trials[trial number][data index][index]=variable
data indexes
0 - time
1 - front lidar
2 - right lidar
3 - back lidar
4 - left lidar
5 - x accel
6 - y accel
7 - pitch
8 - roll
9 - yaw
10 - airspeed
11 - groundspeed
12 - altitude
13 - accel er x
14 - accel er y
15-18 - Lidar rates of change
'''
dts = [(37 - 18) / 147, (74 - 53) / 707, (57 - 38) / 647, (57 - 26) / 465, (69 - 47) / 718]


def getLidVelAt(mat, n, i):
    deltas = []
    dt = dts[i]
    for l in range(4):
        v = 0
        if (n == 0):
            v = (mat[1 + l][n + 1] - mat[1 + l][n]) / dt
        elif (n + 1 == len(mat[0])):
            v = (mat[1 + l][n] - mat[1 + l][n - 1]) / dt
        else:
            v = (mat[1 + l][n] - mat[1 + l][n - 1]) / dt
            v = (v + (mat[1 + l][n + 1] - mat[1 + l][n]) / dt) / 2
        deltas.append(v)
    return deltas

def getAbberation(mat, n,i):
    dt = dts[i]
    dt=1
    v = 0
    if (n == 0):
        v = abs(mat[n + 1] - mat[n]) / dt
    elif (n + 1 == len(mat)):
        v = abs(mat[n] - mat[n - 1]) / dt
    else:
        v = abs(mat[n] - mat[n - 1]) / dt
        v = (v + abs(mat[n + 1] - mat[n]) / dt) / 2
    return v

def smoothAbberant(mat, n,i):
    dt = dts[i]
    v = 0
    if (n == 0):
        v = abs(mat[n + 1] - mat[n]) / dt
    elif (n + 1 == len(mat)):
        v = abs(mat[n] - mat[n - 1]) / dt
    else:
        v = abs(mat[n] - mat[n - 1]) / dt
        v = (v + abs(mat[n + 1] - mat[n]) / dt) / 2
    return v



def smooth(mat):
    size = len(mat)
    last = mat[0]
    newvals = []
    for i in range(size - 1):
        v = (last + mat[i + 1]) / 2.0 * .3 + mat[i] * .7
        last = mat[i]
        newvals.append(v)
    for v in range(len(newvals)):
        mat[v] = newvals[v]
    return mat

def addVelocities(mat, d):
    mat1 = [[], [], [], []]
    for i in range(len(mat[0])):
        vels = getLidVelAt(mat, i, d)
        for n in range(4):
            mat1[n].append(vels[n])
    lim=20
    for set in mat1:
        last=set[0]
        for i in range(len(set)):
            if(set[i]>lim):
                set[i]=lim
            elif(set[i]<-lim):
                set[i]=-lim
    for n in range(4):
        set1=[]
        for i in range(len(mat[1])):
            set1.append(getAbberation(mat1[n],i,n))
        #for t in range(1):
            #mat1[n]=smooth(mat1[n])
        #mat.append(mat1[n])
        mat.append(set1)



trials = []
for i in range(5):
    trials.append(imp.getData(i + 1))
for d in range(5):
    addVelocities(trials[d], d)
    print(trials[d])


def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()


def renderLid():
    globals()
    clear(win)
    print('drawing')
    colors = ["green", "red", "blue", "black"]
    scale = 1
    for s in range(5):
        for v in range(4):
            adj = int(height * float(s + 1) / 6)
            x = 0
            y = trials[s][15 + v][0] * scale
            print(trials[s][15 + v])
            size = len(trials[s][15 + v])
            for n in range(size):
                last = Point(x, adj - y * scale)
                x += width / size
                y = trials[s][15 + v][n]
                this = Point(x, adj - y * scale)
                lin = Line(last, this)
                lin.setFill(colors[v])
                lin.draw(win)
        '''adj = int(height * float(s + 1) / 5)
        x = 0
        y = trials[s][12][0] * 100
        print(trials[s][12])
        size = len(trials[s][12])
        for n in range(size):
            last = Point(x, adj - y * 100)
            x += width / size
            y = trials[s][12][n]
            this = Point(x, adj - y * 100)
            lin = Line(last, this)
            lin.draw(win)'''
        '''for l in range(4):
            x = 0
            y = lid1[0][1 + l]
            size = len(lid1)
                last = Point(x, adj - y * 5)
                x += width / size
                y = lid1[n][1 + l]
                this = Point(x, adj - y * 5)
                lin = Line(last, this)
                lin.setFill(colors[l])
                lin.draw(win)
            data[s][1] = smooth(data[s][1], l + 1)'''


renderLid()
while not keyboard.is_pressed('esc'):
    if (keyboard.is_pressed('r')):
        renderLid()
    time.sleep(2)
