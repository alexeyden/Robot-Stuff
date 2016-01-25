#!env python3

import model
import pid

import tkinter as tk
import numpy as np

class sim:
	def __init__(self):
		self.root = tk.Tk()
		self.canvas = tk.Canvas(self.root, width=800, height=600)
		self.canvas.pack()

		self.whf = self.canvas.create_line(
			0, 0, 0, 0,
			fill = 'red', width = 5
		)
		self.whr = self.canvas.create_line(
			0, 0, 0, 0,
			fill = 'blue', width = 5
		)
		self.chas = self.canvas.create_line(
			0, 0, 0, 0,
			fill = 'black', width = 2.0
		)

		self.text = self.canvas.create_text((10, 10), anchor='nw')
		self.state = self.canvas.create_text((10, 25), anchor='nw')

		self.model = model.model(init = [0, 0, 0, 0, 1, 1, 0])
		
		self.params = tk.Frame(self.root)		
		self.params.pack(fill=tk.X)

		tk.Label(self.params, text='res_k').grid(row=0, column=1)
		self.para_res_k = tk.Scale(self.params, orient='h', from_ = 0, to = 0.01, resolution=0.0005)
		self.para_res_k.grid(row=1, column=1)
		self.para_res_k.set(self.model.res_k)

		tk.Button(self.params, text='reset', command=self._reset).grid(row=0, column=0)

		self.prev_y = None

		self.draw_scale = 25
		self.time = 0
		self.dt = 30/1000
		
		self.pause = False 

		self.root.bind('<space>', self.__pause)

	def _reset(self):
		self.model = model.model(init = [0, 0, 0, 0, 1, 1, 0])
		self.time = 0
		self.prev_y = None
		self.canvas.delete('tracer')

	def _draw(self, y):
		v_x, v_y, r, s_f, x, y, o, *u = y

		k = self.draw_scale

		Of = [ x * k + 0.75 * k * np.cos(o), y * k + 0.75 * k * np.sin(o) ]
		Or = [ x * k - 0.75 * k * np.cos(o), y * k - 0.75 * k * np.sin(o) ]

		if self.prev_y:
			self.canvas.create_line(*self.prev_y[0], x * k, y * k, fill='black', tags=('tracer'))
			self.canvas.create_line(*self.prev_y[1], *Of, fill='red', tags=('tracer'))
			self.canvas.create_line(*self.prev_y[2], *Or, fill='blue', tags=('tracer'))

		self.canvas.coords(self.whf, [
			Of[0] + 0.25 * k * np.cos(o + s_f),
			Of[1] + 0.25 * k * np.sin(o + s_f),
			Of[0] - 0.25 * k * np.cos(o + s_f),
			Of[1] - 0.25 * k * np.sin(o + s_f)
		])

		self.canvas.coords(self.whr, [
			Or[0] + 0.25 * k * np.cos(o),
			Or[1] + 0.25 * k * np.sin(o),
			Or[0] - 0.25 * k * np.cos(o),
			Or[1] - 0.25 * k * np.sin(o)
			]
		)

		self.canvas.coords(self.chas, [Of[0], Of[1], Or[0], Or[1]])
		self.prev_y = ((x * k, y * k), Of, Or)
	
	def __pause(self, ev):
		self.pause = not self.pause

	def __update(self):
		if self.pause:
			self.root.after(30, self.__update)
			return

		self.time += self.dt

		y = self.model.simulate(self.dt, self._control)

		self._draw(y)

		self.canvas.itemconfig(self.text, text='{0:.2f}'.format(self.time))
		self.canvas.itemconfig(self.state,
				text = 'v=({0:.2f},{1:.2f}); r={2:.2f}; s={3:.2f}; xy=({4:.2f},{5:.2f}); o={6:.2f}; u_v={7:.2f}, u_s={8:.2f} pause={9}'.format(*y, self.pause))

		self.model.res_k = self.para_res_k.get()

		self.root.after(30, self.__update)
	
	def _control(self, t, y):
		raise NotImplemented()
	
	def run(self):
		self.__update()
		self.root.mainloop()
