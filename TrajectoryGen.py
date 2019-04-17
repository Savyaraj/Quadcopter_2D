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

class node():
	def __init__(self,dim):
		self.dim = dim     #dimension of state space
		self.p = np.zeros((dim,2))
		self.control = np.zeros((1,2))

	def open(self):
		p_temp = self.p
		return tuple(p_temp.reshape(1,2*self.dim)[0])

	def __lt__(self,other):
		return True

class graph():
	def __init__(self,dim,map,render,lim,resolution,rho):
		self.render = render
		self.map = map
		self.dim = dim
		self.start = node(dim)
		self.start.p[0] = map.start
		self.goal = node(dim)
		self.goal.p[0] = map.goal
		self.lim = lim
		self.x_max = lim[0]
		self.v_max = lim[1]
		self.a_max = lim[2]
		self.u_max = lim[3]
		self.tau = resolution[0]
		self.cont_range = np.linspace(-lim[dim],lim[dim],resolution[1])
		self.du = 2*lim[dim]*resolution[1]
		self.rho = rho

	def is_feasible(self,state):       #calculate theta 
									   #check collision at intermediate points
									   #closed form solutions for v_max and a_max
		dyn_feasible = True
		for i in range(state.dim):
			for j in range(2):
				if abs(state.p[i][j])>=self.lim[i]: 
					dyn_feasible = dyn_feasible and False

		collision = self.map.is_colliding(state.p[0])
		return dyn_feasible and not collision

	def neighbors(self,current):       #use motion primitives to calculate neighboring nodes
		neighbor_list = []
		for ux,uy in itertools.product(self.cont_range,self.cont_range):
			u = np.array(([ux,uy]))
			next = node(self.dim)
			next.control = u
			if self.dim == 1:
				next.p[0] = current.p[0]+self.tau*u

			if self.dim == 2:
				next.p[1] = current.p[1]+self.tau*u
				next.p[0] = current.p[0] + current.p[1]*self.tau + (u*self.tau**2)/2

			if self.dim == 3:
				next.p[2] = current.p[2]+u*self.tau
				next.p[1] = current.p[1]+current.p[2]*self.tau+u*(self.tau**2)/2
				next.p[0] = current.p[0]+current.p[1]*self.tau+(current.p[2]*self.tau**2)/2+(u*self.tau**3)/6

			if self.is_feasible(next):
				neighbor_list.append(next)

		return neighbor_list


	def cost(self,current,next):          #cost for the trajectory with given end points
									      #the sum of control efforts and time taken
		return (np.linalg.norm(next.control)**2+self.rho)*self.tau							  

	def heuristic(self,current,goal):     #use QMVT heuristic
		h1 = np.linalg.norm(current.p[0]-goal.p[0])/self.lim[1]
		return self.rho*h1

	def kts(self,key):
		state = node(self.dim)
		state.p = np.array(key).reshape(self.dim,2)
		return state

	def A_star_search(self,start,goal):   

		start_key = start.open()
		goal_key = goal.open()
		frontier = PriorityQueue()
		frontier.put(start, 0)
		came_from = {}
		cost_so_far = {}
		came_from[start_key] = None
		cost_so_far[start_key] = 0
		iter = 0
		
		while not frontier.empty():
			current = frontier.get()
			if iter%500 ==0 and came_from[current.open()] != None:
				self.render.draw_point(current.p[0],'r',1)
				prev = came_from[current.open()]
				self.render.draw_line(current.p[0],prev.p[0],'r')
				theta = math.atan((current.p[0][1]-prev.p[0][1])/(current.p[0][0]-prev.p[0][0]))
				v = np.linalg.norm(current.p[1])
				temp = current.p[0]+[-(v/self.v_max)*math.sin(theta),(v/self.v_max)*math.cos(theta)]
				self.render.draw_line(current.p[0],temp,'b')
			iter=iter+1
			print(iter)
			if (np.linalg.norm(current.p[0]-goal.p[0])<1) or iter==10000:
				break

			for next in graph.neighbors(self,current):
				new_cost = cost_so_far[current.open()] + graph.cost(self,current,next)
				if next.open() not in cost_so_far or new_cost < cost_so_far[next.open()]:
					cost_so_far[next.open()] = new_cost
					priority = new_cost + graph.heuristic(self,next,goal)
					print(iter)
					frontier.put(next,priority)
					came_from[next.open()] = current
				
		return came_from, cost_so_far

