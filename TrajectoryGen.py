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
from basics import Properties
from Bot import bot 
from maps import *


class graph():
	def __init__(map):
		self.map = map

	def neighbors(current):           #use motion primitives to calculate neighboring nodes


	def cost(current, next):          #cost for the trajectory with given end points
									  #the sum of control efforts and time taken



class trajectory():
	def __init__(map,constraints,resolution,rho):
		self.map = map
		self.start = map.start
		self.goal = map.goal
		self.constraints = constraints
		self.tau = resolution[0]
		self.du = 2*constraints[2]*resolution[1]
		self.rho = rho

	def heuristic(self,start,goal):
		h1 = np.linalg.norm(np.array(start-goal))/self.constraints[0]
		return h1

	def is_feasible(map,constraints):
		return True
		
	def A_star_search(graph,start,goal):


