import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm, trange

shape_data = np.genfromtxt('shapes.csv', delimiter=',')
# shape_data = shape_data[shape_data[:, 3] == 278]

# print(shape_data)

# plt.fill()

# for i in range(183):
#     lane_pts = shape_data[shape_data[:, 0] == i]
#     if (len(lane_pts) == 0):
#         continue
#     print(lane_pts)
#     if lane_pts[0, 3] == 278:
#         plt.fill(lane_pts[:, 1], lane_pts[:, 2], c='purple')
#     else:
#         plt.fill(lane_pts[:, 1], lane_pts[:, 2], c='gray')

# plt.show()

data = np.genfromtxt('grid.csv', delimiter=',')

plt.imshow(data)

# occupied = data[data[:, 2] == 1]
# plt.scatter(occupied[:, 0], occupied[:, 1])

plt.show()
