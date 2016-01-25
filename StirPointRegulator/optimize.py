#!env python3
# -*- coding: utf-8 -*-
import numpy as np

from util import *
from pid import *
from model import *

import sys

tmax = 240
dt = 0.1

points = [(0, 0), (20, 30), (10, -20), (-40, 0), (-15, -15)]
points_x = [0] + [x[0] for x in points]
points_y = [0] + [x[1] for x in points]
v_abs = 1
	
init = [
	0.0, 0, #vx, vy
	0, 0, #r, s
	0, 0, #x, y
	0 #o
]

def solve(x, *args):
	p,d = x
	data_x = []
	data_y = []
	m = model(init=init)
		
	L = 0
	pt = None
	
	pid_dir = pid(kp = p, kd = d, ki = 0)
	pid_abs = pid(kp = 50, kd = 20, ki = 0.1)
	
	p = points[:]

	def control(t, y):
			v_x, v_y, r, s_f, x, y, o, *u = y
			
			e_v = v_abs - np.linalg.norm([v_x, v_y])
			e_s = vec_angle([np.cos(o), np.sin(o)], [p[0][0] - x, p[0][1] - y])	

			u_v = pid_abs.update(e_v, t)
			u_s = pid_dir.update(e_s, t)

			return (u_v, u_s)

	while m.ir.successful() and m.ir.t < tmax and len(p) > 0: 
		y = m.simulate(dt, control)
		v_x, v_y, r, s_f, x, y, o, *u = y
		
		data_x.append(x)
		data_y.append(y)
		if not pt is  None:
			L += np.linalg.norm([x - pt[4], y - pt[5]]) 

		pt = m.ir.y

		if np.linalg.norm([x - p[0][0], y - p[0][1]]) < 1:
			p.remove(p[0])
		
	assert(m.ir.successful())
	
	return (L, m.ir.t, data_x,data_y)

def show():
	L1,t1, dx1, dy1 = solve([0.6, 1.5])

	import matplotlib.pyplot as plt
	from mpl_toolkits.mplot3d import Axes3D

	import matplotlib
	matplotlib.rc('font', family='Arial')

	plt.title('Траектория (x,y)')
	plt.xlabel('Расстояние (м)')
	plt.ylabel('Расстояние (м)')

	plt.plot(points_x, points_y, 'ro', label='Точки маршрута')
	
	text1 = '$K_p={:.2f}, K_d={:.2f}, L={:.2f} м$'.format(0.60, 1.50, L1)

	plt.plot(dx1, dy1, label=text1, color='black')

	plt.legend(bbox_to_anchor=(0.4, 0.25))
	plt.show()

def optimize():
	import scipy.optimize as opt
	bounds = [slice(0.1, 20, 1), slice(0, 20, 1)]
	# rs = opt.brute(lambda v: solve(v)[0], bounds, full_output=True)
	bounds = [(0.1, 20), (0, 20)]
	rs = opt.differential_evolution(lambda v: solve(v)[0], bounds)

	print(rs)

locals()[sys.argv[1]]()
