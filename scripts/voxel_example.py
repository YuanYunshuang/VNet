import tkinter
import matplotlib.pyplot as plt
import numpy as np
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

N1 = 10
N2 = 10
N3 = 10
ma = np.random.choice([0,1], size=(N1,N2,N3), p=[0.99, 0.01])
print(ma)


def midpoints(x):
	sl = ()
	for i in range(x.ndim):
		#print(sl + np.index_exp[:-1])
		#print(sl + np.index_exp[:1])
		#print(x)
		#print("===========================")
		#a=x[sl + np.index_exp[:-1]]
		#print(a)
		#print("===========================")
		#b=x[sl + np.index_exp[1:]]
		#print(b)
		#print("===========================")
		x = (x[sl + np.index_exp[:-1]]+x[sl + np.index_exp[1:]]) / 2.0 # using

		sl += np.index_exp[:]
	#print(x)
	return x

# prepare some coordinates, and attach rgb values to each
r, g, b = np.indices((5, 5, 5)) / 4.0
#print(np.indices((3, 3, 3)))
rc = midpoints(r)
#print(rc)
gc = midpoints(g)
#print(gc)
bc = midpoints(b)

# define a sphere about [0.5, 0.5, 0.5]
sphere = (rc - 0.5)**2 + (gc - 0.5)**2 + (bc - 0.5)**2 < 0.5**2
print(sphere)
# combine the color components
colors = np.zeros(sphere.shape + (3,))
colors[..., 0] = rc
colors[..., 1] = gc
colors[..., 2] = bc

# and plot everything
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(r, g, b, sphere,
          facecolors=colors,
          edgecolors=np.clip(2*colors - 0.5, 0, 1),  # brighter
          linewidth=0.5)
ax.set(xlabel='r', ylabel='g', zlabel='b')

plt.show()
