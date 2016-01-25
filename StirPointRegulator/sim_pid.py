#!env python3

import tkinter as tk
import numpy as np

from sim import sim
from pid import pid
from util import *

class sim_pid(sim):
	def __init__(self):	
		super().__init__()
		
		self.root.bind('<Button-1>', self.__click)

		self.point = None
		self.init_k_s = [1, 5, 0]
		self.init_k_v = [50, 20, 0.1]
		self.init_v = 3.0

		self.pid_s = pid(*self.init_k_s)
		self.pid_v = pid(*self.init_k_v)

		self.para_kp_s = tk.StringVar()
		self.para_kd_s = tk.StringVar()
		self.para_kp_s.set(self.init_k_s[0])
		self.para_kd_s.set(self.init_k_s[1])

		self.para_kp_v = tk.StringVar()
		self.para_kd_v = tk.StringVar()
		self.para_ki_v = tk.StringVar()
		self.para_kp_v.set(self.init_k_v[0])
		self.para_kd_v.set(self.init_k_v[1])
		self.para_ki_v.set(self.init_k_v[2])

		self.para_v = tk.StringVar()
		self.para_v.set(self.init_v)

		tk.Label(self.params, text='Kp (s)').grid(row=0, column=2)
		tk.Entry(self.params, width=5, textvariable=self.para_kp_s).grid(row=1, column=2)

		tk.Label(self.params, text='Kd (s)').grid(row=0, column=3)
		tk.Entry(self.params, width=5, textvariable=self.para_kd_s).grid(row=1, column=3)

		tk.Label(self.params, text='Kp (v)').grid(row=0, column=4)
		tk.Entry(self.params, width=5, textvariable=self.para_kp_v).grid(row=1, column=4)

		tk.Label(self.params, text='Kd (v)').grid(row=0, column=5)
		tk.Entry(self.params, width=5, textvariable=self.para_kd_v).grid(row=1, column=5)

		tk.Label(self.params, text='Ki (v)').grid(row=0, column=6)
		tk.Entry(self.params, width=5, textvariable=self.para_ki_v).grid(row=1, column=6)

		tk.Label(self.params, text='v').grid(row=0, column=7)
		tk.Entry(self.params, width=5, textvariable=self.para_v).grid(row=1, column=7)

		self.text_e = self.canvas.create_text((10, 40), anchor='nw')

		self.point_p = self.canvas.create_oval([0, 0, 0, 0], fill='red', state=tk.HIDDEN) 

	def __click(self, ev):
		v_x, v_y, r, s_f, x, y, o = self.model.ir.y
		p = (ev.x/self.draw_scale, ev.y/self.draw_scale)
		self.point = p
		self.canvas.itemconfig(self.point_p, state=tk.NORMAL)
	
	def _draw(self, y):
		super()._draw(y)

		if self.point: 
			k = self.draw_scale
			self.canvas.coords(self.point_p, [
				self.point[0]*k - 5, self.point[1]*k - 5,
				self.point[0]*k + 5, self.point[1]*k + 5,
			])
	
	def _reset(self):
		super()._reset()
		self.point = None
		self.canvas.itemconfig(self.point_p, state=tk.HIDDEN)

		self.pid_s = pid(*self.init_k_s)	
		self.pid_v = pid(*self.init_k_v)	
		
	def _control(self, t, y):
		v_x, v_y, r, s_f, x, y, o = y

		self.pid_s.kp = float(self.para_kp_s.get())
		self.pid_s.kd = float(self.para_kd_s.get())

		self.pid_v.kp = float(self.para_kp_v.get())
		self.pid_v.kd = float(self.para_kd_v.get())
		self.pid_v.ki = float(self.para_ki_v.get())

		e_v = float(self.para_v.get()) - np.linalg.norm([v_x, v_y])
		e_s = vec_angle([np.cos(o), np.sin(o)], [self.point[0] - x, self.point[1] - y])	if not self.point is None else 0	
	
		u_v = self.pid_v.update(e_v, t)
		u_s = self.pid_s.update(e_s, t)

		self.canvas.itemconfig(self.text_e, text='e_s={0:.1f} ({1:.2f}) e_v={2:.2f} u_s={3:.2f} (real)'.format(np.degrees(e_s), e_s, e_v, u_s))

		return (u_v, u_s)

sim_pid().run()
