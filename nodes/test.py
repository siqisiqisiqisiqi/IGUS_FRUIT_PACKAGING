import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import yaml
import numpy as np
from numpy.linalg import norm


threshold = 50
box_type = "Type1"
path_file = f"{box_type}.yaml"
with open(f"{parent}/config/{path_file}", "r") as f:
    box_info = yaml.safe_load(f)
position = box_info["Position"]
capacity = box_info["Capacity"][0]

# print(f"position is {position}")
# print(f"capacity is {capacity}")

p_p_array = []
peach_position = [-180, 60, 110]
p_p_array.append(peach_position)
peach_position = [-80, 60, 110]
p_p_array.append(peach_position)
peach_position = [-65, 80, 110]
p_p_array.append(peach_position)
peach_position = [20, 50, 110]
p_p_array.append(peach_position)
pp_array = np.array(p_p_array)
print(pp_array[0])

empty_position = [x+1 for x in range(capacity)]

pp_xy_array = pp_array
for key,value in position.items():
    xy = value[:2]
    dist = norm(pp_xy_array[:,:2] - xy, axis = 1)
    dist_min = np.min(dist)
    if dist_min < threshold:
        idx_peach = np.argmin(dist)
        empty_position.remove(key)
        pp_xy_array = np.delete(pp_xy_array, (idx_peach), axis=0)
print(pp_xy_array)
print(pp_array)