import numpy as np
from basics import Properties
import math

init = Properties()
[L,M,I,g] = [init.L,init.M,init.I,init.g]

class bot:
	Trajectory = []
	def __init__(self, Length = L, Mass = M, Inertia = I, time = 0):
		self.L = Length
		self.M = Mass
		self.I = Inertia
		self.t = time

	def set(self, x = 0, y = 0, theta = 0, vx = 0, vy = 0, w = 0, t = 0):
		self.loc = [x, y]
		self.theta = theta
		self.v = [vx, vy]
		self.w = w
		self.t = t
		self.Trajectory.append(np.array([self.loc[0], self.loc[1], self.theta, self.v[0], self.v[1], self.w, self.t]))

		


