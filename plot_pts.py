import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt('points.csv', delimiter=',')
print(data)

plt.scatter(data[:, 0], data[:, 1])
plt.show()
