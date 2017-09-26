import numpy as np
import matplotlib.pyplot as plt

N = 50
x = np.random.rand(N)
y = np.random.rand(N)

plt.scatter(x, y)
plt.show()

data = np.array([
    [1, 2],
    [2, 3],
    [3, 6],
])
x, y = data.T
plt.scatter(x,y)

plt.show()