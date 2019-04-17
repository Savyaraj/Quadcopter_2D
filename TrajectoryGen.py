#Trajectory Generation
#Defined as      [x(t),y(t)] 

# input: 1] obstacle map
#        2] current bot position and goal position
#        2] dynamic constraints for the bot    [v_max,a_max,u_max]
#        3] graph resolution for time t and motion primitives du     [tau,du/(2*u_max)]
#        4] cost function parameters [rho]

# output:   
#        1] motion primitive value and trajectory for the next time step
#        2] cost corrsponding to the step
#        3] complete trajectory and the cost associated at the end of the optimization
#        4] examples of sub-optimal trajectories for comparison

# Motion Primitive:
#      each motion primitive is a cubic polynomial wrt time for time interval [0,tau]

#      p(t) = p0 + v0*t + (a0/2)*t^2 + (u/6)*t^3

#      where [p0,v0,a0] -> initial state
#                    u  -> constant jerk input 

# Heuristic:
#      H(s,sg) = min C(T)
#      minimum cost from current state to the goal with unconstrained optimization

# feasibility: to be completed


import numpy as np 
from maps import *
import itertools
import heapq
import matplotlib.pyplot as plt
from graphics import Renderer

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

class graph():
	def __init__(self,map,render,lim,resolution,rho):
		self.render = render
		self.map = map
		self.start = np.array((map.start,[0,0],[0,0],[0,0]))
		self.goal = np.array((map.goal,[0,0],[0,0],[0,0]))
		self.lim = lim
		self.v_max = lim[0]
		self.a_max = lim[1]
		self.u_max = lim[2]
		self.tau = resolution[0]
		self.u_range = np.linspace(-self.u_max,self.u_max,resolution[1])
		self.du = 2*self.u_max*resolution[1]
		self.rho = rho

	def is_feasible(self,state):       #calculate theta 
									   #check collision at intermediate points
									   #closed form solutions for v_max and a_max

		dyn = ((state[2]<=self.v_max) and (state[3]<=self.v_max) and (state[4]<=self.a_max) and (state[5]<=self.a_max))	
		collision = self.map.is_colliding(state[0],state[1])
		#print(collision)
		return dyn and not collision

	def neighbors(self,current):       #use motion primitives to calculate neighboring nodes
		neighbor_list = []
		current = (np.array(current)).reshape(4,2)
		for ux,uy in itertools.product(self.u_range,self.u_range):
			u = np.array(([ux,uy]))
			a = current[2]+u*self.tau
			v = current[1]+current[2]*self.tau+(self.tau**2/2)*u
			s = current[0]+current[1]*self.tau+current[2]*self.tau**2/2+u*self.tau**3/6
			next = tuple((np.array([s,v,a,u])).reshape(1,8)[0])
			if self.is_feasible(next):
				neighbor_list.append(next)

		return neighbor_list


	def cost(self,current,next):          #cost for the trajectory with given end points
									      #the sum of control efforts and time taken
		return (np.linalg.norm([next[6],next[7]])**2+self.rho)*self.tau							  

	def heuristic(self,current,goal):     #use QMVT heuristic
		h1 = np.linalg.norm([current[0]-goal[0],current[1]-goal[1]])/self.lim[0]
		return self.rho*h1

	def A_star_search(self,start,goal):   

		start = tuple(start.reshape(1,8)[0])
		goal = tuple(goal.reshape(1,8)[0])
		frontier = PriorityQueue()
		frontier.put(start, 0)
		came_from = {}
		cost_so_far = {}
		came_from[start] = None
		cost_so_far[start] = 0
		iter = 0

		# fig = plt.figure()
		# plt.clf()
		# l = self.map.map_size[1]
		# plt.axis([-l,l,-l,l])
		# plt.xlabel('$x(m)$', Fontsize = 16)
		# plt.ylabel('$y(m)$', Fontsize = 16)
		# plt.xticks(Fontsize = 16)
		# plt.yticks(Fontsize = 16)
		# plt.plot(start[0],start[1], marker="o",  markersize=5)
		# plt.plot(goal[0],goal[1], marker="o",  markersize=5)
		
		while not frontier.empty():
			current = frontier.get()
			if iter%1000 ==0:
				self.render.draw_point([current[0],current[1]],'r',1)	
			iter=iter+1
			print(iter)
			if (np.linalg.norm([current[0]-goal[0],current[1]-goal[1]])<0.1) or iter==100000:
				path = [current]
				print(current)
				while current in came_from:
					current = came_from[current]
					path.append(current)
				path.reverse()
				break

			for next in graph.neighbors(self,current):
				new_cost = cost_so_far[current] + graph.cost(self,current,next)
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far[next] = new_cost
					priority = new_cost + graph.heuristic(self,next,goal)
					frontier.put(next,priority)
					came_from[next] = current
					# if iter%1000 == 0:
						# self.render.draw_line([current[0],current[1]],[next[0],next[1]],'r')	
		# 				plt.plot([current[0],next[0]],[current[1],next[1]],color='red', linewidth=1)
		# 				plt.show(block=False)
		# 				plt.pause(0.01)
		# plt.savefig('Test.png')
		# plt.pause(10)
		return came_from, cost_so_far, path

