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

trials = []
for i in range(5):
    trials.append(imp.getData(i + 1))
for d in trials:
    print(d)

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
'''


def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

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


'''while not keyboard.is_pressed('esc'):
    # if (keyboard.is_pressed('r')):
    renderLid()
    time.sleep(2)'''

'''
def renderLid():
    globals()
    clear(win)
    print('drawing')
    colors = ["green", "red", "blue", "black"]
    for s in range(5):
        adj = int(height * float(s + 1) / 5)
        lid1 = data[s][1]
        for l in range(4):
            x = 0
            y = lid1[0][1 + l]
            size = len(lid1)
            for n in range(size):
                last = Point(x, adj - y * 5)
                x += width / size
                y = lid1[n][1 + l]
                this = Point(x, adj - y * 5)
                lin = Line(last, this)
                lin.setFill(colors[l])
                lin.draw(win)
            data[s][1] = smooth(data[s][1], l + 1)
'''