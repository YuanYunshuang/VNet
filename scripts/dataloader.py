# ====Autor: Yuan. Yunshuang====
# ====Date: Dec. 30. 2018=======


# This is a class loading RGB point clouds and generate training or test samples,
# in format of a 4d matrix (s,s,s,3),
# s==> is the number of voxels of the cube size
# 3: RGB chanels

# each voxel has the size of v(m)
# we set default of s to 40, and v to 0.003, which means we only condiser the
# data in a space range of [-0.06,0.06]m in x , y direction, and [0,1.2]m in
# z direction.
import tkinter
import matplotlib.pyplot as plt
import numpy as np

# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

class DataReader:
	def __init__(self, s=40, v=0.003):
		self.s = s
		self.v = v
		offset = self.v/2.0 # avoid index out of range
		self.x_min = -0.06 + offset
		self.x_max = 0.06 - offset
		self.z_max = 0.12 - offset

	#@staticmethod
	def loadTxt(self, filename):
		obj = []
		for line in open(filename):
			#lines = f.readlines()
			obj.append(line.split(' '))

		return np.array(obj, dtype=np.float64)

	#@staticmethod
	def isBB(self, point): # check if the point is in bounding box
		if point[0]<self.x_min or point[0]>self.x_max:
			return False
		if point[1]<self.x_min or point[1]>self.x_max:
			return False
		if point[2]<0.0 or point[2]>self.z_max:
			return False

		return True

	#@staticmethod
	def getSam(self,filename): # generate sample data format for the object
		obj = self.loadTxt(filename)
		ocp_grid = np.zeros((self.s,self.s, self.s,3),dtype=np.float64) # initializing occupancy ocp_grid
		mean_color = np.mean(obj[:,3:]/255.0, axis=0)
		# print(mean_color.shape)
		for i in range(obj.shape[0]):
			if self.isBB(obj[i,:]):
				x = int(round(obj[i,0]/self.v) + self.s/2)
				y = int(round(obj[i,1]/self.v) + self.s/2)
				z = int(round(obj[i,2]/self.v))
				ocp_grid[x,y,z] = obj[i,3:] - mean_color

		return ocp_grid

	#@staticmethod
	def viewData(self,ocp_grid):
		ocp_grid = ocp_grid/255.0
		#r, g, b = np.indices((ocp_grid.shape[:-1])) * 0.003
		cube = ocp_grid[...,1]>0.0


		fig = plt.figure()
		ax = fig.gca(projection='3d')
		ax.voxels(cube,
		  facecolors=ocp_grid,
		  edgecolors=np.clip(2*ocp_grid - 0.5, 0, 1),
		  linewidth=0.05)
		ax.set(xlabel='x', ylabel='y', zlabel='z')
		plt.show()

"""
# test

dl = DataReader()
filename = "/home/ophelia/data/txt_cleansed/01_pc0.txt"
ocp_grid = dl.getSam(filename)
ocp_grid = ocp_grid/255.0
r, g, b = np.indices((ocp_grid.shape[:-1])) * 0.003
print(r.shape)
cube = ocp_grid[...,1]>0.0
print(cube.shape)


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(cube,
          facecolors=ocp_grid,
          edgecolors=np.clip(2*ocp_grid - 0.5, 0, 1),
          linewidth=0.05)
ax.set(xlabel='r', ylabel='g', zlabel='b')

plt.show()

"""
