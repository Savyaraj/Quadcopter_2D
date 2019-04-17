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
#map1.add_rectangle(0	,-20,20,20)
v = Renderer(map1)
v.draw_point(map1.start,'b',5)
v.draw_point(map1.goal,'g',5)
plt.pause(1)
g = graph(2,map1,v,[5,5,10,50],[0.1,8],10000)

# list = g.neighbors(g.start)
# print(list[0].p)
# print(g.cost(g.start,list[0]))
# print(g.heuristic(g.start,list[0]))

came_from,b,path = g.A_star_search(g.start,g.goal)

for current in came_from:
	print(a[current])
	prev = came_from[current.open()]
	v.draw_point(current.p[0],'r',1)
	theta = math.atan((current.p[0][1]-prev.p[0][1])/(current.p[0][0]-prev.p[0][0]))
	vel = np.linalg.norm(current.p[1])
	print(vel)
	temp = current.p[0]+[-(vel/g.v_max)*math.sin(theta),(vel/g.v_max)*math.cos(theta)]
	v.draw_line(current.p[0],temp,'b')
plt.pause(10)