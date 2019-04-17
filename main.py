import numpy as np
import properties
from Bot import bot
from graphics import Renderer
from Integrator import Integrator
import matplotlib.pyplot as plt
import math
from maps import *
from TrajectoryGen import graph

# b = bot()
# b.set(10*L,5*L,0.5)
# v = Visualizer()
# Integrator(b,10)

map1 = obstacle_map([-30,30],[-20,-20],[25,10])
map1.add_rectangle(-17,-20,20,20)
v = Renderer(map1)
v.draw_point(map1.start,'b',5)
v.draw_point(map1.goal,'g',5)
plt.pause(1)
g = graph(map1,v,[7,10,50],[0.2,5],10000)

# list = g.neighbors(g.start)
# print(list[0])
# print(g.cost(g.start,list[0]))
# print(g.heuristic(g.start,g.goal))

a,b,path = g.A_star_search(g.start,g.goal)
plt.pause(10)

# path.remove(None)
# x = []
# y = []
# for current in path:
# 	x = np.append(x,current[0])
# 	y = np.append(y,current[1])
# print(path)