import numpy as np
c = []
a = np.array([[3],[1],[-1]])
b = a.squeeze()
c.append(b)
c.append(b)
c.append(b)
c.append(b)
c = np.array(c)
for i in c:
    print(i)