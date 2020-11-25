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


def getData(num):
    fname = "dronedata\drone" + str(num)
    lidfname = fname + "lid.txt"
    fname = fname + ".txt"
    dat_file = open(fname, 'r')
    lid_file = open(lidfname, 'r')
    dat_raw = dat_file.read()
    lid_raw = lid_file.read()
    lid = reverse(csvToMatrix(lid_raw))
    dat = csvToMatrix(dat_raw)
    print(lid)
    print(dat)
    return [dat, lid]
