import pandas as pd
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
from mpl_toolkits import mplot3d


MMM = pd.read_csv('darren testing/MMM.csv')
points = (MMM[["vehicle_accelerations_NTB_0","vehicle_accelerations_NTB_1","s_dot",]].dropna().to_numpy())
print(points)
print(np.shape(points))
hull = ConvexHull(points)


# import matplotlib.pyplot as plt
# plt.plot(points[:,0], points[:,1], 'o')
#
# for simplex in hull.simplices:
#     plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
# plt.show()


import matplotlib.pyplot as plt
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(points[:,0], points[:,1], points[:,2], 'gray')
ax.set_xlabel("vehicle_accelerations_NTB_0")
ax.set_ylabel("vehicle_accelerations_NTB_1")
ax.set_zlabel("s_dot")

for simplex in hull.simplices:
    ax.scatter3D(points[simplex,0], points[simplex,1], points[simplex,2], 'k-')
print(len(hull.simplices))
plt.show()
