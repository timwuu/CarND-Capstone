from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

length_end_s = 90.0

ptsx = []
ptsy = []

ptsx.append(1261.38)
ptsy.append(1191.83)
ptsx.append(1262.38)
ptsy.append(1191.81)
ptsx.append(1280.70)
ptsy.append(1191.35)
ptsx.append(1297.28)
ptsy.append(1190.57)
ptsx.append(1313.81)
ptsy.append(1189.17)
ptsx.append(1330.16)
ptsy.append(1187.43)
ptsx.append(1346.49)
ptsy.append(1185.81)

spline_s = [-1.0, 0, length_end_s*0.333, length_end_s*0.5, length_end_s*0.667, length_end_s*0.833, length_end_s]

pts = []
for i in range(0, len(ptsx)):
    pts.append([ptsx[i],ptsy[i]])
cs = interpolate.CubicSpline(spline_s, pts)
xs = np.linspace(0, length_end_s, 30)
plt.figure(figsize = (6.5, 4))
plt.plot(cs(xs)[:, 0], cs(xs)[:, 1], label='spline')
plt.plot(ptsx, ptsy)
plt.show()