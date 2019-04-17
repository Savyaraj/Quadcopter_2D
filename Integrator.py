import numpy as np 
import math
from scipy import integrate
from basics import Properties
from Int_func import Int_func
import matplotlib.pyplot as plt 
from graphics import Renderer
from Bot import bot

init = Properties()
[L,M,I,g] = [init.L,init.M,init.I, init.g]

def Integrator(bot, T = 20, dt = 0.1, Method = 'lsoda'):
	y0 = np.array([bot.loc[0],bot.loc[1],bot.theta, bot.v[0],bot.v[1], bot.w])
	sol = integrate.ode(Int_func).set_integrator(Method)
	sol.set_initial_value(y0,0)
	v = Visualizer()
	while sol.successful() and sol.t<T:
		step = np.array(sol.integrate(sol.t+dt))
		bot.set(*np.append(step,sol.t))
		v.plot(bot)

	plt.plot(bot.Trajectory[0],bot.Trajectory[2])

                