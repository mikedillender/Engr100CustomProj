import math
import numpy as np


def csvToMatrix(csv):
    lines = []
    cstr = ""
    for c in csv:
        if (c == '\n' or c == '\r'):
            lines.append(cstr)
            cstr = ""
        else:
            cstr = cstr + c
    mat = []
    for l in lines:
        cmat = []
        cstr = ""
        for c in l:
            if (c == ','):
                if (len(cstr.replace(" ", "")) > 0):
                    cmat.append(float(cstr.replace(" ", "")))

                cstr = ""
            else:
                cstr = cstr + c
        if len(cstr.replace(" ", "")) > 0:
            cmat.append(float(cstr.replace(" ", "")))
        mat.append(cmat)
    return mat


def reverse(mat):
    mat1 = []
    siz = len(mat)
    for i in range(siz):
        mat1.append(mat[siz - i - 1])
    return mat1


def invert(mat):
    mat1 = []
    wid = len(mat)
    hei = len(mat[0])
    for y in range(hei):
        vals = []
        for x in range(wid):
            vals.append(mat[x][y])
        mat1.append(vals)
    return mat1


def resize(mat, s):
    c = len(mat)
    mat1 = []
    for i in range(s):
        percent = float(i) / s
        exactind = percent * c
        decimal = exactind - math.floor(exactind)
        val = 0
        if (math.ceil(exactind) >= c):
            val = mat[math.floor(exactind)]
        else:
            val = mat[math.floor(exactind)] * (1 - decimal) + (decimal) * mat[math.ceil(exactind)]
        mat1.append(val)
    return mat1

def trim(mat):
    print("trim new")
    alt = 12-4
    start = 0
    last=[0,0,0]
    for m in mat[alt]:
        last[start%3]=m
        print("cutting "+str(m)+", start = "+str(start))
        if np.matrix(last).mean() > .2:
            break
        start = start + 1
    mat1 = []
    wid = len(mat)
    leng = len(mat[0])
    trim=leng-start
    for n in range(wid):
        newarr=[]
        for i in range(trim):
            newarr.append(mat[n][i+start])
        mat1.append(newarr)
    return mat1



def getData(num):
    fname = "dronedata\drone" + str(num)
    lidfname = fname + "lid.txt"
    fname = fname + ".txt"
    dat_file = open(fname, 'r')
    lid_file = open(lidfname, 'r')
    dat_raw = dat_file.read()
    lid_raw = lid_file.read()
    lid = invert(reverse(csvToMatrix(lid_raw)))
    dat = trim(invert(csvToMatrix(dat_raw)))
    #dat = (invert(csvToMatrix(dat_raw)))
    if (len(lid[0]) < len(dat[0])):
        for i in range(5):
            lid[i] = resize(lid[i], len(dat[0]))
    else:
        for i in range(11):
            dat[i] = resize(dat[i], len(lid[0]))
    #print(lid)
    #print(dat)
    full = []
    full.append(dat[0])
    for i in range(4):
        full.append(lid[i + 1])
    for i in range(10):
        full.append(dat[i + 1])
    return full
