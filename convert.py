def pcd_to_pts(input,output):
    line = input.readline()
    while line:
        try:
            numbers = [float(a) for a in line.split(" ")[0:6]]
            output.write(" ".join([str(a) for a in numbers]) + "\n")
        except:
            pass
        line = input.readline()
    output.close()

if __name__ == "__main__":
    pcd_to_pts(open("output.pcd"),open("test.pts","w"))
