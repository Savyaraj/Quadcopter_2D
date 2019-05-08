import numpy as np
from graphics import Renderer
import matplotlib.pyplot as plt
import math
from scipy import integrate
from maps import *
from TrajectoryGen import *
g = 9.81
L = 1
c = 0.00
M = 0.5
I = 0.01
class controller:
	def __init__(self,graph,reference,vdes):
		self.graph = graph
		self.start = graph.start
		self.ref = reference
		self.traj = []
		self.vdes = vdes

	def integrator(self,dt,T,Method):
		start = np.array([self.start.p[0][0],self.start.p[0][1],0,0,0,0])
		t = 0
		current = start
		while (t<T):
			t = t+dt
			current = current + self.updates(current)*dt
			current_node = node(self.graph.dim)
			current_node.p[0] = [current[0],current[1]]
			current_node.theta = current[2]
			self.traj.append(current_node)

	def updates(self,state):
		Kp = [2, 1.5, 0.1]
		Kd = [1.5, 2, 1.5]
		er = self.track_error(state)
		if(np.linalg.norm(er)!=0):
			normal = er/np.linalg.norm(er)
		else:
			normal = er

		if normal[1]<0.2:
			normal = [0,1]
		ev = [self.vdes*abs(normal[1])-state[3],self.vdes*abs(normal[0])-state[4]];
		[ax,ay] = [Kp[0]*er[0]+Kd[0]*ev[0],Kp[1]*er[1]+Kd[1]*ev[1]]
		#Linear Dynamics

		Ta = M*(ay+g)
		theta_des = M*ax/Ta
		if theta_des>math.pi/6:
			theta_des=math.pi/6
		if theta_des<-math.pi/6:
			theta_des=-math.pi/6

		ert= state[2]-theta_des
		evt = -state[5]
		Ts = Kp[2]*ert+Kd[2]*evt

		if state[5]>math.pi/6:
			state[5]=math.pi/6
		if state[5]<-math.pi/6:
			state[5]=-math.pi/6

		return np.array([state[3], state[4], state[5], (Ta/M)*math.sin(state[2])-c*state[3], (Ta/M)*math.cos(state[2])-g-c*state[4], (Ts/I)*(L/2)-c*state[5]])

	def track_error(self,current_state):
		min_dist = self.graph.lim[0]
		closest = self.ref[0].p[0]
		index = 0
		current = np.array([current_state[0],current_state[1]])
		for i in range(len(self.ref)):
			state = self.ref[i]
			if(np.linalg.norm(current-state.p[0])<min_dist):
				closest = state.p[0]
				index = i
				min_dist = np.linalg.norm(current-closest)
				print(closest)
				print(min_dist)
		if index == len(self.ref)-1:
			index = index - 1
		closest = self.ref[index+1].p[0]
		print(closest)
		er = closest-[current[0],current[1]]
		return er

	def render(self):
		for i in range(len(self.traj)-1):
			if i%10 != 0:
				continue
			self.graph.render.draw_line(self.traj[i].p[0],self.traj[i+10].p[0],'g')
			self.graph.render.draw_point(self.traj[i].p[0],'b',5)
			s = self.traj[i]
			Pl = [s.p[0][0]-(L/2)*math.cos(s.theta),s.p[0][1]+(L/2)*math.sin(s.theta)]
			Pr = [s.p[0][0]+(L/2)*math.cos(s.theta),s.p[0][1]-(L/2)*math.sin(s.theta)]
			self.graph.render.draw_line(Pl,Pr,'k')
			plt.pause(0.01)
		plt.pause(5)
		return