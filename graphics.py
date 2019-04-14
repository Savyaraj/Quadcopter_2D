import numpy as np
from basics import Properties
from matplotlib import pyplot as plt
from matplotlib import axes 
import math
from decimal import Decimal
from Bot import bot

init = Properties()
[L,M,I,g] = [init.L,init.M,init.I, init.g]

class Visualizer:

	fig = plt.figure()
	def plot(self, bot):
		plt.clf()
		x = bot.loc[0]
		y = bot.loc[1]
		theta = bot.theta
		t = bot.t
		plt.axis([x-L, x+L, y-L, y+L])
		plt.xlabel('$x(m)$', Fontsize = 16)
		plt.ylabel('$y(m)$', Fontsize = 16)
		plt.xticks(Fontsize = 16)
		plt.yticks(Fontsize = 16)
		p1 = [x-(L/2)*math.cos(theta),y+(L/2)*math.sin(theta)]
		p2 = [x+(L/2)*math.cos(theta),y-(L/2)*math.sin(theta)]
		plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='black', linewidth=2)
		plt.text(x+0.4*L, y+0.4*L, '$t = $'+str(round(t,4))+' $s$'+'\n'+'$x/L = $'+str(round(x/L,4))+'\n'+'$y/L = $' +str(round(y,4))+'\n'+'$\\theta = $' +str(round(theta*180/math.pi,4))+'$^0$', Fontsize = 16)
		plt.show(block=False)
		plt.pause(0.01)




		