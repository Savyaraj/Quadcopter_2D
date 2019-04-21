# 2-D maps for trajectory generation
import numpy as np 
import math


class free_map():
	def __init__(self,size,start,goal):
		self.map_size = np.array(size)
		self.start = np.array(start)
		self.goal = np.array(goal)


class obstacle_map():
	def __init__(self,size,start,goal):
		self.map_size = np.array(size)
		self.start = np.array(start)
		self.goal = np.array(goal)
		self.c = 0.5
		self.obstacles = {
			"Rectangle":[],
			"Circle":[]
		}                      #obstacle array

	def add_rectangle(self,x,y,l,w):             # (x,y): left corner , (l,w): length,width
		self.obstacles["Rectangle"].append([x,y,l,w])
		return

	def add_circle(self,x,y,r):
		self.obstacles["Cicle"].append([x,y,r])
		return

	def is_colliding(self,p):

		size = self.map_size[1]
		if abs(p[0])>size-self.c or abs(p[1])>size-self.c:
			return True

		for R in self.obstacles['Rectangle']:
			if((R[0]-self.c<=p[0] and (p[0]<=R[0]+R[2]+self.c)) and (R[1]-self.c<=p[1] and (p[1]<=R[1]+R[3]+self.c))):
				return True

		return False

