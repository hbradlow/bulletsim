import re
p = open("test.pcd")
n = open("normals.pcd")
pn = open("pointnormals.pcd","w+")

p_line = p.readline()
n_line = n.readline()
i = 0
if True:
    pn.write("VERSION 0.7\n")
    pn.write("FIELDS x y z normal_x normal_y normal_z\n")
    pn.write("SIZE 4 4 4 4 4 4\n")
    pn.write("TYPE F F F F F F\n")
    pn.write("COUNT 1 1 1 1 1 1\n")
while p_line and n_line:
    i+=1
    p_match = re.match(r'^(.*?)\s+(.*?)\s+(.*?)\s+(.*?)$',p_line)
    n_match = re.match(r'^(.*?)\s+(.*?)\s+(.*?)\s+(.*?)$',n_line)
    try:
        l = [float(p_match.group(1)),float(p_match.group(2)),float(p_match.group(3)),float(n_match.group(1)),float(n_match.group(2)),float(n_match.group(3))]
        pn.write(" ".join([str(a) for a in l]) + "\n")
    except:
        if i>6:
            pn.write(p_line)
    p_line = p.readline()
    n_line = n.readline()
pn.close()
