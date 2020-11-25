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

data = []
for i in range(5):
    print("data " + str(i + 1) + " : ")
    data.append(imp.getData(i + 1))


def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()


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


def smooth(mat, ind):
    size = len(mat)
    last = mat[0][ind]
    newvals = []
    for i in range(size - 1):
        v = (last + mat[i + 1][ind]) / 2.0 * .3 + mat[i][ind] * .7
        last = mat[i][ind]
        newvals.append(v)
    for v in range(len(newvals)):
        mat[v][ind] = newvals[v]
    return mat


'''while not keyboard.is_pressed('esc'):
    # if (keyboard.is_pressed('r')):
    renderLid()
    time.sleep(2)'''
