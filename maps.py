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

	def is_colliding(self,x,y):

		for R in self.obstacles['Rectangle']:
			if((R[0]<=x and (x<=R[0]+R[2]))):
				return True

			if((R[1]<=y and (y<=R[1]+R[3]))):
				return True	

		return False
	# def is_colliding(self,x,y,L,theta):

	# 	r = [x+(L/2)*math.cos(theta), y-(L/2)*math.sin(theta)]
	# 	l = [x-(L/2)*math.cos(theta), y+(L/2)*math.sin(theta)]

	# 	for R in self.obstacles['Rectangle']:
	# 		if(((R[0]-R[2]/2)<=r[0] and (r[0]<=R[0]+R[2]/2)) or ((R[0]-R[2]/2)<=l[0] and (l[0]<=R[0]+R[2]/2))):
	# 			return True

	# 		if(((R[1]-R[3]/2)<=r[1] and (r[1]<=R[1]+R[3]/2)) or ((R[1]-R[3]/2)<=l[1] and (l[1]<=R[1]+R[3]/2))):
	# 			return True	