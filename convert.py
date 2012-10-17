f = open("output.pcd")
out = open("test.pts","w")
line = f.readline()
while line:
    try:
        numbers = [float(a) for a in line.split(" ")[0:6]]
        out.write(" ".join([str(a) for a in numbers]) + "\n")
    except:
        pass
    line = f.readline()
out.close()
