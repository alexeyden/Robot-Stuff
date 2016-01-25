from scipy import integrate as sciint
import numpy as np
from scipy import signal

class model:
	def __init__(self, init=None, noise_s=None, noise_v=None):
		self.C_af = self.C_ar = 5.4/(np.pi/180)
		self.L_f = self.L_r = 0.75
		self.W = 1
		self.m = 100
		self.I_z = self.m/12 * (self.W**2 + (self.L_f + self.L_r)**2)
		self.res_k = 0.001
		self.F_res = lambda: self.res_k/0.20 * self.m * 9.8

		self.init = init
		
		self._uv = None
		self._us = None
		
		self.noise_s = None
		self.noise_v = None

		self.max_s = np.pi/4
		self.max_u_s = np.pi/2
		self.max_u_v = 3 * 100
		
		self.reset()

	def reset(self):
		self.ir = sciint.ode(self.__step).set_integrator('vode', atol=0.001)
		self.ir.set_initial_value(self.init, 0)

	def simulate(self, dt, control):
		assert(self.ir.successful())
		
		self.ir.set_f_params(dt, control)
		self.ir.integrate(self.ir.t + dt)
		
		return list(self.ir.y) + [self._uv, self._us]

	def __step(self, t, y, dt, control):
		u_v, u_s = control(t, y)
		v_x, v_y, r, s_f, x, y, o = y

		a_f = (v_y + self.L_f * r) / (v_x + 0.00001) - s_f
		a_r = (v_y - self.L_r * r) / (v_x + 0.00001) 

		F_yf = -self.C_af * a_f
		F_yr = -self.C_ar * a_r
		
		n_v = self.noise_v(t) if self.noise_v else 0
		n_s = self.noise_s(t) if self.noise_s else 0
		
		u_s = u_s if abs(u_s) < self.max_u_s else np.copysign(self.max_u_s, u_s)
		u_v = u_v if abs(u_v) < self.max_u_v else np.copysign(self.max_u_v, u_v)
		
		if abs(s_f + u_s * dt) > self.max_s:
			u_s = np.copysign(self.max_s - 0.05, s_f) - s_f
		
		self._uv = u_v
		self._us = u_s
		
		F_res = self.F_res() * v_x
		
		d_v_x = (-F_yf * np.sin(s_f) + u_v - F_res * np.cos(s_f) - F_res + n_v) / self.m + v_y * r
		d_v_y = (F_yf * np.cos(s_f) + F_yr - F_res * np.sin(s_f)) / self.m - v_x * r
		d_r   =  self.L_f * (F_yf * np.cos(s_f) - F_res * np.sin(s_f)) / self.I_z - self.L_r * F_yr/self.I_z + n_s
		d_s   = u_s
		d_x = v_x * np.cos(o) - v_y * np.sin(o)
		d_y = v_x * np.sin(o) + v_y * np.cos(o)
		d_o = r

		return [
			d_v_x, d_v_y, d_r,
			d_s, d_x, d_y, d_o
		]
			
