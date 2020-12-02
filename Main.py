import Importing as imp
from graphics import *
import keyboard
import time
import sys
import numpy as np

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
19-forward/back lidv
20-right/left lidv
21-lidar-based x
22-lidar-based y
'''
dts = [(37 - 18) / 147, (74 - 53) / 707, (57 - 38) / 647, (57 - 26) / 465, (69 - 47) / 718]
lps = []
trials = []


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


def getAbberation(mat, n):
    dt = 1
    v = 0
    if (n == 0):
        v = abs(mat[n + 1] - mat[n]) / dt
    elif (n + 1 == len(mat)):
        v = abs(mat[n] - mat[n - 1]) / dt
    else:
        v = abs(mat[n] - mat[n - 1]) / dt
        v = (v + abs(mat[n + 1] - mat[n]) / dt) / 2
    return v


def averageAboutN(mat, n, s,d):
    sz = len(mat)
    num = mat[n]
    denom = 1
    powr = 2
    for i in range(s-1):
        b = n - i-1
        f = n + i-1
        s1 = pow(1-((i + 1) / (s)), powr)
        if b >= 0:
            num = num + (mat[b] * s1)
            denom = denom + s1
        if f < sz:
            num = num + (mat[b] * s1)
            denom = denom + s1
    return num / denom
    #return mat[n]*.8


def smoothAbberant(mat,d):
    mat1 = []
    sz = len(mat)
    dtype = [('v', float), ('i', int)]
    for i in range(len(mat)):
        mat1.append((getAbberation(mat, i), i))
    arr = np.array(mat1, dtype=dtype)
    # print(arr)
    arr = np.sort(arr, order='v')
    tosmooth = int(sz / 50)
    aboutN=[4,4,4,4,4]
    #print(str(aboutN[d]))
    for i in range(tosmooth):
        # print(arr[sz - 1 - i][0])
        if (arr[sz - 1 - i][0] > 2):
            #mat[arr[sz - 1 - i][1]] = averageAboutN(mat, arr[sz - 1 - i][1], aboutN[d],d)
            if(mat[arr[sz - 1 - i][1]]!=0):
                mat[arr[sz - 1 - i][1]] =mat[arr[sz - 1 - i][1]]*abs((abs(mat[arr[sz - 1 - i][1]])-1)/abs(mat[arr[sz - 1 - i][1]]))
            else:
                mat[arr[sz - 1 - i][1]] = averageAboutN(mat, arr[sz - 1 - i][1], aboutN[d], d)

    return True


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
    lim = 20
    for set in mat1:
        last = set[0]
        for i in range(len(set)):
            if (set[i] > lim):
                set[i] = lim
            elif (set[i] < -lim):
                set[i] = -lim
    for n in range(4):
        set1 = []
        r = True
        for i in range(100):
            r = smoothAbberant(mat1[n],d)
            if (not r):
                break
        #mat1[n] = smooth(mat1[n])
        mat.append(mat1[n])


def getNetVel(mat, d, i):  # d=0 : front / back, d=1 : right / left
    ld = [mat[1 + d][i], mat[3 + d][i]]
    lv = [-mat[15 + d][i], mat[17 + d][i]]
    weight = .5
    if (lv[0] == 0):
        return lv[1]
    elif (lv[1] == 0):
        return lv[0]
    return (lv[0] + lv[1]) / 2


def constructNetVels():
    for mat in trials:
        sz = len(mat[0])
        for d in range(2):
            d1 = []
            for i in range(sz):
                d1.append(getNetVel(mat, d, i))
            mat.append(d1)


def getPositions():
    t = 0
    scales = [7, 10, 10, 8, 10]
    for mat in trials:
        sz = len(mat[0])
        for d in range(2):
            d1 = []
            x = 0
            for i in range(sz):
                dt0 = .01
                d1.append(x)
                x = x + mat[19 + d][i] * dts[t] * scales[t]
                # x=x+mat[19+d][i]*mat[0][i]*dts[t]*30
            d1.append(x)
            mat.append(d1)
        t += 1


def getLidPositions():
    global lps
    t = 0
    scales = [7.5, 6.8, 6, 7, 7.5]
    mults = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    for mat in trials:
        sz = len(mat[0])
        ps = []
        for i in range(sz):
            dr = Point(mat[21][i], mat[22][i])
            for d in range(4):
                if mat[1 + d][i] < 14:
                    ps.append(Point(dr.x + mat[1 + d][i] * mults[d][0] * scales[t],
                                    dr.y + mat[1 + d][i] * mults[d][1] * scales[t]))
        lps.append(ps)
        t += 1


def cleanLPS():
    global lps
    for t in lps:
        rem=[]
        for i in range(len(t)):
            closest=10000
            for n1 in range(len(t)-1-i):
                n=i+n1+1
                dx=t[i].x-t[n].x
                dy=t[i].y-t[n].y
                d=np.sqrt(dx*dx+dy*dy)
                if (d<closest):
                    closest=d
            if(closest>3):
                rem.append(i)
        print("removed "+str(len(rem)))
        for i in range(len(rem)):
            t.pop(rem[i]-i)


def setup():
    global trials, lps
    for i in range(5):
        trials.append(imp.getData(i + 1))
    for d in range(5):
        addVelocities(trials[d], d)
        # print(trials[d])
    constructNetVels()
    getPositions()
    getLidPositions()
    #cleanLPS()
    print("done")


def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

setup()

def renderLid():
    globals()
    clear(win)
    print('drawing')
    colors = ["green", "red", "blue", "black", "brown"]
    adjxs = [width / 4, width - width / 4, width / 2, width / 4, width - width / 4, ]
    adjys = [height / 3 - 100, height / 3 - 100, height / 2 - 50, 2 * height / 3, 2 * height / 3]

    mode=2
    scale = 5 if mode==0 else 1
    if(mode==2):
        for s in range(5):
            adjy = adjys[s] + 100
            adjx = adjxs[s] - 100
            for p in lps[s]:
                pt = Point(p.x + adjx, p.y + adjy)
                pt.setFill(colors[s])
                pt.draw(win)

    for s in range(5):
        if mode==1 or mode==2:
            # adjy = int(height * float(s + 1) / 6)
            # adjx = int(width/2)
            adjy = adjys[s] + 100
            adjx = adjxs[s] - 100
            x = trials[s][21][0]
            y = trials[s][22][0]
            size = len(trials[s][21])
            for n in range(size):
                last = Point(adjx + x * scale, adjy + y * scale)
                x = trials[s][21][n]
                y = trials[s][22][n]
                this = Point(adjx + x * scale, adjy + y * scale)
                lin = Line(last, this)
                lin.setFill(colors[s])
                lin.draw(win)

        if mode==0:
            for v in range(2):
                adj = int(height * float(s + 1) / 6)
                zro=Line(Point(0,adj),Point(width,adj))
                zro.draw(win)
                x = 0
                y = trials[s][19 + v][0] * scale
                print(trials[s][19 + v])
                size = len(trials[s][19 + v])
                for n in range(size):
                    last = Point(x, adj - y * scale)
                    x += width / size
                    y = trials[s][19 + v][n]
                    this = Point(x, adj - y * scale)
                    lin = Line(last, this)
                    lin.setFill(colors[v])
                    lin.draw(win)
        '''
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
        '''
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
