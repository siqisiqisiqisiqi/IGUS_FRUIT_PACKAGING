import pickle
import matplotlib.pyplot as plt

# with open("test3", "rb") as fp:   # Unpickling
#     b = pickle.load(fp)
# with open("time", "rb") as fp:   # Unpickling
#     t = pickle.load(fp)
# x1 = b[80]
# x2 = b[140]
# t1 = t[80]
# t2 = t[140]
# vel = (x2-x1)/(t2-t1)
# print(vel)
t1 = 3.5
x1 = 10
t2 = 6.17
x2 = 30
t3 = 8.82
x3 = 50
t4 = 11.41
x4 = 90
v1 = (x2-x1)/(t2-t1)
v2 = (x3-x2)/(t3-t2)
v3 = (x3-x2)/(t3-t2)
print(v1)
print(v2)
print(v3)
