#Trajectory Generation
#Defined as      [x(t),y(t)] 

# input: 1] obstacle map
#        2] current bot position and goal position
#        2] dynamic limits for the bot
#        3] graph resolution for time t and motion primitives du
#        4] cost function parameters (rho)

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

# feasibility:


import numpy as np 
from basics import Properties
from Bot import bot 
from maps import obstacle_map