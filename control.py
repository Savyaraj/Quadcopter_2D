import numpy as np
from graphics import Renderer
import matplotlib.pyplot as plt
import math
from maps import *
from TrajectoryGen import *

class controller:
	def __init__(self,graph,reference):
		self.graph = graph
		self.start = graph.start
		self.ref = reference
		self.traj = []

	def integrator(self,dt,T,Method):
		sol = integrate.ode(updates).set_integrator(Method)
		start = [self.start.p[0][0],self.start.p[0][1],0,0,0,0]
		sol.set_initial_value(start,0)
		while sol.successful() and sol.t<T:
			step = np.array(sol.integrate(sol.t+dt))
			np.append(self.traj,step)

			
	def PID(self,state,error):
		Kp = [1, 1, 1]
		Kd = [1.8, 1.8, 1.8]
		[ax,ay] = [Kp[0]*er[0]+Kd[0]*ev[0],Kp[1]*er[1]+Kd[1]*ev[1]]
		#Linear Dynamics
		theta_des = ax/(ay+g)
		er[2] = state[4]-theta_des
		ev[2] = -state[5]
		atheta = Kp[2]*er[2]+Kd[2]*ev[2]
		return [state[3], state[4], state[5], ax, ay+g, atheta]


	def render(self):
		

	def error_estimate(self):



