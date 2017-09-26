from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

length_end_s = 90.0

ptsx = []
ptsy = []

ptsx.append(1130.22)
ptsy.append(1183.18)
ptsx.append(1131.22)
ptsy.append(1183.27)
ptsx.append(1163.48)
ptsy.append(1189.75)
ptsx.append(1193.53)
ptsy.append(1191.17)
ptsx.append(1223.63)
ptsy.append(1192.43)


spline_s = [-1.0, 0, length_end_s*0.333, length_end_s*0.667, length_end_s]

pts = []
for i in range(0, len(ptsx)):
    pts.append([ptsx[i],ptsy[i]])
cs = interpolate.CubicSpline(spline_s, pts)
xs = np.linspace(0, length_end_s, 30)
plt.figure(figsize = (6.5, 4))
plt.plot(cs(xs)[:, 0], cs(xs)[:, 1], label='spline')
plt.plot(ptsx, ptsy)
plt.show()