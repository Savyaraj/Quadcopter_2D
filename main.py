import numpy as np
from basics import Properties
from Bot import bot
from graphics import Visualizer
from Integrator import Integrator
import matplotlib.pyplot as plt
import math
from maps import *
from TrajectoryGen import graph

init = Properties()
[L,M,I,g] = [init.L,init.M,init.I, init.g]
# b = bot()
# b.set(10*L,5*L,0.5)
# v = Visualizer()
# Integrator(b,10)

map1 = free_map([-100*L,100*L],[0,0],[50*L,50*L])
g = graph(map1,[7,10,50],[0.2,10],10000)
# list = g.neighbors(g.start)
# print(list[0])
# print(g.cost(g.start,list[0]))
# print(g.heuristic(g.start,g.goal))

# plt.figure()
# plt.clf()
l = map1.map_size[1]
plt.axis([-l,l,-l,l])
plt.xlabel('$x(m)$', Fontsize = 16)
plt.ylabel('$y(m)$', Fontsize = 16)
plt.xticks(Fontsize = 16)
plt.yticks(Fontsize = 16)
plt.plot(map1.start[0],map1.start[1], marker="o",  markersize=5)
plt.plot(map1.goal[0],map1.goal[1], marker="o",  markersize=5)

a,b,path = g.A_star_search(g.start,g.goal)
path.remove(None)
x = []
y = []
for current in path:
	x = np.append(x,current[0])
	y = np.append(y,current[1])
print(path)

plt.plot(x,y,color='red', linewidth=1)
plt.show()
plt.pause(10)

# a = np.array([[1,1],[0,0],[2,3],[4,5]])
# b = tuple(a.reshape(1,8)[0])
# print(b)
# dic = {}
# dic[b] = 'haha'
# print(dic[b])
# c = np.array(b)
# c = c.reshape(4,2)
# print(np.array(c))
