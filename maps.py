# 2-D maps for trajectory generation
import numpy as np 


class free_map():
	def __init__(self,size,start,goal):
		self.map_size = np.array(size)
		self.start = np.array(start)
		self.goal = np.array(goal)


class obstacle_map():
	def __init__(size,start,goal):
		self.map_size = np.array(size)
		self.start = np.array(start)
		self.goal = np.array(goal)
		self.obstacles = []                      #obstacle array
	def add_rectangle(self,x,y,l,w):

	def add_circle(self,x,y,r):

	def is_colliding(self,x,y,theta):