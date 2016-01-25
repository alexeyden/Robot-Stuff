#!env python3

from sim import sim

class sim_hand(sim):
	def __init__(self):
		super().__init__()

		self.root.bind('<Up>', self.__keydown)
		self.root.bind('<KeyRelease-Up>', self.__keyup)

		self.root.bind('<Left>', self.__keydown)
		self.root.bind('<KeyRelease-Left>', self.__keyup)

		self.root.bind('<Right>', self.__keydown)
		self.root.bind('<KeyRelease-Right>', self.__keyup)

		self.keys = {'left':False, 'right':False, 'up':False, 'break':False}

	def _control(self, t, y):
		u_v = self.model.max_u_v if self.keys['up'] is True else 0

		u_s = 0
		if self.keys['left']: u_s = -self.model.max_u_s
		elif self.keys['right']: u_s = +self.model.max_u_s

		return (u_v, u_s)
		
	def __keyup(self, ev):
		if ev.keysym == 'Left': self.keys['left'] = False
		if ev.keysym == 'Right': self.keys['right'] = False
		if ev.keysym == 'Up': self.keys['up'] = False
		if ev.keysym == 'Space': self.keys['break'] = False

	def __keydown(self, ev):
		if ev.keysym == 'Left': self.keys['left'] = True
		if ev.keysym == 'Right': self.keys['right'] = True 
		if ev.keysym == 'Up': self.keys['up'] = True
		if ev.keysym == 'Space': self.keys['break'] = True

sim_hand().run()
