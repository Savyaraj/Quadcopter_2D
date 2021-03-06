import numpy as np
import properties
from Bot import bot
from graphics import Renderer
from Integrator import Integrator
import matplotlib.pyplot as plt
import math
from maps import *
from TrajectoryGen import *

map1 = obstacle_map([-8,8],[-7,-7],[-5,6])
map1.add_rectangle(-8,-3,5,2)
map1.add_rectangle(2,2,4,3)
# map1.add_rectangle(1,2,2,2)
v = Renderer(map1)
dim = 1
lim = [8,7,7,20]
rho = 15*lim[dim]**2        # rho = 4 for vel control
	 					   #       2 for acc control
resolution = [0.2,6]
heuristic_method = 'distance'
iterations = 100000
accuracy = 1

g = graph(dim,map1,v,lim,resolution,rho,[])    #5 bins for acc, 8 for vel
came_from,cost_so_far,current = g.A_star_search(g.start,g.goal,iterations,accuracy)
v.close()
v = Renderer(map1)
g.plot_trajectory()

rho = 15*lim[dim+1]**2        # rho = 4 for vel control
	 					   #       2 for acc control
resolution = [0.2,6]
g1 = graph(dim+1,map1,v,lim,resolution,rho,g.read_data('Trajectory.txt'))
came_from,cost_so_far,current = g1.A_star_search(g1.start,g1.goal,iterations,accuracy)
v.close()
v = Renderer(map1)
g1.plot_trajectory()

rho = 15*lim[dim+2]**2
g2 = graph(dim+2,map1,v,lim,resolution,rho,g1.read_data('Trajectory.txt'))#,g.read_data('Trajectory.txt'))
came_from,cost_so_far,current = g2.A_star_search(g2.start,g2.goal,iterations,accuracy)
v.close()
v = Renderer(map1)
g2.plot_trajectory()

v.close()
v = Renderer(map1)
g.plot_trajectory()
g1.plot_trajectory()
g2.plot_trajectory()
