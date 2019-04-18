import numpy as np
import properties
from Bot import bot
from graphics import Renderer
from Integrator import Integrator
import matplotlib.pyplot as plt
import math
from maps import *
from TrajectoryGen import *

map1 = obstacle_map([-5,5],[-2,-2],[3,4])
map1.add_rectangle(0,0,2,2)
v = Renderer(map1)
v.draw_point(map1.start,'b',5)
v.draw_point(map1.goal,'g',5)
dim = 2
lim = [5,7,10,50]
rho = 10*lim[dim]**2
g = graph(dim,map1,v,lim,[0.2,5],rho)

# list = g.neighbors(g.start)
# print(list[0].p)
# print(g.cost(g.start,list[0]))
# print(g.heuristic(g.start,list[0]))

# came_from,cost_so_far,current = g.A_star_search(g.start,g.goal)
# g.write_data()
g1 = graph(dim,map1,v,lim,[0.2,4],rho)
g1.read_data("came_from.txt")
came_from = g1.camefrom 
current = g1.last

while came_from[current.open()] != None:
	prev = came_from[current.open()]
	v.draw_line(current.p[0],prev.p[0],'r')
	if current == None or prev == None:
		continue
	theta = math.atan((current.p[0][1]-prev.p[0][1])/(current.p[0][0]-prev.p[0][0]))
	vel = np.linalg.norm(current.p[1])
	temp = current.p[0]+[-(vel/g1.v_max)*math.sin(theta),(vel/g1.v_max)*math.cos(theta)]
	v.draw_line(current.p[0],temp,'b')
	current = prev
plt.pause(10)