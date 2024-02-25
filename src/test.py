#!/usr/bin/env python3
import numpy as np
from numpy.linalg import norm
# from dynamic_grasp import constant_jerk_kinematic_model

# target = np.array([150, 0, 100])
# home = np.array([0, 0, 200])
# vel = 400
# t_sum = constant_jerk_kinematic_model(target, home, vel, offset=50)
# print(t_sum)

# peach_array = np.array([[1,2,3], [4,5,6], [7,8,9], [1,1,1]])
# for i in peach_array:
#     print(i)
pp_xy_array = np.array([[-88.10310006,68.13124853,30.10188293]])
xy = [-180, 70]
print(pp_xy_array.shape)
dist = norm(pp_xy_array[:, :2] - xy, axis=1)
dist_min = np.min(dist)
pp_xy_array = np.delete(pp_xy_array, (0), axis=0)
print(pp_xy_array.shape[0])