from basics import Properties
import numpy as np 

init = Properties()
[L,M,I,g] = [init.L,init.M,init.I, init.g]

def Int_func(t,y):
	Kp = [1, 1, 1]
	Kd = [1.8, 1.8, 1.8]
	er = [-y[0], -y[1], -y[2]]
	ev = [-y[3], -y[4], -y[5]]
	[ax,ay] = [Kp[0]*er[0]+Kd[0]*ev[0],Kp[1]*er[1]+Kd[1]*ev[1]]
	theta_des = ax/(ay+g)
	atheta = Kp[2]*er[2]+Kd[2]*ev[2]
	return [y[3], y[4], y[5], ax, ay, atheta]
