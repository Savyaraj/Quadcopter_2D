import numpy as np
from graphics import Renderer
import matplotlib.pyplot as plt
import math
from maps import *
from TrajectoryGen import *

class controller:
	def __init__(self,map,trajectory):
		self.map = map
		self.ref = trajectory

	def integrator(self):
		sol = integrate.ode(PID).set_integrator(Method)
		sol.set_initial_value(y0,0)
		while sol.successful() and sol.t<T:
			step = np.array(sol.integrate(sol.t+dt))
			
	def PID(self):
		Kp = [1, 1, 1]
		Kd = [1.8, 1.8, 1.8]
		er = [-y[0], -y[1], -y[2]]
		ev = [-y[3], -y[4], -y[5]]
		[ax,ay] = [Kp[0]*er[0]+Kd[0]*ev[0],Kp[1]*er[1]+Kd[1]*ev[1]]
		theta_des = ax/(ay+g)
		atheta = Kp[2]*er[2]+Kd[2]*ev[2]
		return [y[3], y[4], y[5], ax, ay, atheta]

	def render(self):
		