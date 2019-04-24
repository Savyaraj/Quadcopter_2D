import numpy as np
import properties
from matplotlib import pyplot as plt
from matplotlib import axes 
import matplotlib.patches as patches
import math
from decimal import Decimal
from Bot import bot

class Renderer:
	def __init__(self,map):

		fig,ax = plt.subplots(1)
		self.map = map
		# plt.grid(True)
		l = map.map_size[1]
		plt.axis([-l,l,-l,l])
		plt.xlabel('$x(m)$', Fontsize = 16)
		plt.ylabel('$y(m)$', Fontsize = 16)
		plt.xticks(Fontsize = 16)
		plt.yticks(Fontsize = 16)
		plt.locator_params(nbins=10)
		self.draw_point(self.map.start,'b',40)
		self.draw_point(self.map.goal,'g',40)

		for R in map.obstacles["Rectangle"]:

			rect = patches.Rectangle((R[0],R[1]),R[2],R[3],linewidth=1,edgecolor='k',facecolor='k')
			ax.add_patch(rect)

	def draw_line(self,p1,p2,col):

		plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color=col,linewidth=1)
		plt.draw()
		plt.pause(0.01)

	def draw_point(self,p,col,size):
		plt.plot(p[0],p[1],marker="o",color=col,markersize=size,markeredgewidth=0.0,alpha = 0.3)
		plt.pause(0.01)
		plt.draw()
		# plt.pause(0.01)

	def close(self):
		plt.close()

	# def plot(self, bot):
	# 	plt.clf()
	# 	x = bot.loc[0]
	# 	y = bot.loc[1]
	# 	theta = bot.theta
	# 	t = bot.t
	# 	p1 = [x-(L/2)*math.cos(theta),y+(L/2)*math.sin(theta)]
	# 	p2 = [x+(L/2)*math.cos(theta),y-(L/2)*math.sin(theta)]
	# 	plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='black', linewidth=2)
	# 	plt.text(x+0.4*L, y+0.4*L, '$t = $'+str(round(t,4))+' $s$'+'\n'+'$x/L = $'+str(round(x/L,4))+'\n'+'$y/L = $' +str(round(y,4))+'\n'+'$\\theta = $' +str(round(theta*180/math.pi,4))+'$^0$', Fontsize = 16)
	# 	plt.show(block=False)
	# 	plt.pause(0.01)

