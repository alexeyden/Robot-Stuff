class pid:
	def __init__(self, kp, kd, ki):
		self.kp = kp
		self.kd = kd
		self.ki = ki

		self.N = 20

		self.reset()

	def reset(self):
		self.t = None
		self.e = None
		self.ui = 0
		self.ud = 0
		
	def update(self, e, t):
		if self.t == None:
			self.t = t
			self.e = e
			return 0
		
		dt = t - self.t

		self.ui = self.ui + dt/self.ki * e if self.ki != 0 else 0
		
		self.ud = self.kd/(self.kd + self.N * dt) * self.ud + \
				 self.N * self.kd/(self.kd + self.N * dt) * (e - self.e) if self.kd != 0 else 0

		u = self.kp * e + self.ud + self.ui

		self.t = t
		self.e = e
		
		return u
