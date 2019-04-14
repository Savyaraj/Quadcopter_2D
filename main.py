import numpy as np
from basics import Properties
from Bot import bot
from graphics import Visualizer
from Integrator import Integrator
import matplotlib.pyplot as plt
import math
from maps import *

# init = Properties()
# [L,M,I,g] = [init.L,init.M,init.I, init.g]
# b = bot()
# b.set(10*L,5*L,0.5)
# v = Visualizer()
# Integrator(b,10)

map1 = free_map([-10*L,10*L],[0,0],[5*L,5*L])


