from scipy import integrate as sciint
import numpy as np

from mockup import util


class Robot:
    def __init__(self, position, angle):
        self.state = [0, 0, 0, 0, *position, angle]
        self._model = MovementModel(self.state)

        self.name = ''

        self.plan = []
        self.pid = PID(*[1, 5, 0])

        self.cam_angle = 0.0
        self.cam_width = np.pi / 3

        self.domain = []

        self.width = 0.6
        self.height = 1.0

    def update(self, dt):
        v_x, v_y, *_ = self.state
        if np.linalg.norm([v_x, v_y]) < 0.01 and self.plan == []:
            return
        self.state = self._model.simulate(dt, self._control)

    @property
    def position(self):
        v_x, v_y, r, s_f, x, y, o = self.state
        return [x, y]

    @property
    def angle(self):
        v_x, v_y, r, s_f, x, y, o = self.state
        return o

    @property
    def wheels_angle(self):
        v_x, v_y, r, s_f, x, y, o = self.state
        return s_f

    def _control(self, t, y):
        v_x, v_y, r, s_f, x, y, o = y
        v = np.linalg.norm([v_x, v_y])

        if len(self.plan) == 0:
            return 0.0 if v < 0.01 else -100, 0

        px, py, *_ = self.plan[0]
        dist = np.linalg.norm([x - px, y - py])
        reached_dest = dist < 0.8

        if reached_dest:
            self.plan.pop(0)

        e_v = 20
        e_s = util.vec_angle([np.cos(o), np.sin(o)], [px - x, py - y]) if not reached_dest else 0

        u_v = e_v
        u_s = self.pid.update(e_s, t)

        return u_v, u_s


class PID:
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

        self.ui = self.ui + dt / self.ki * e if self.ki != 0 else 0

        self.ud = self.kd / (self.kd + self.N * dt) * self.ud + \
                  self.N * self.kd / (self.kd + self.N * dt) * (e - self.e) if self.kd != 0 else 0

        u = self.kp * e + self.ud + self.ui

        self.t = t
        self.e = e

        return u


class MovementModel:
    def __init__(self, init=None):
        self.C_af = self.C_ar = 5.4 / (np.pi / 180)
        self.L_f = self.L_r = 0.75
        self.W = 1
        self.m = 100
        self.I_z = self.m / 12 * (self.W ** 2 + (self.L_f + self.L_r) ** 2)
        self.res_k = 0.001
        self.F_res = lambda: self.res_k / 0.20 * self.m * 9.8

        self.init = init

        self._uv = None
        self._us = None

        self.noise_s = None
        self.noise_v = None

        self.max_s = np.pi / 4
        self.max_u_s = np.pi / 2
        self.max_u_v = 3 * 100

        self.ir = sciint.ode(self._step).set_integrator('dopri5', atol=0.0001)
        self.reset()

    def reset(self, t=0.0):
        self.ir.set_initial_value(self.init, t)

    def simulate(self, dt, control):
        assert (self.ir.successful())

        self.ir.set_f_params(dt, control)
        self.ir.integrate(self.ir.t + dt)

        return self.ir.y

    def _step(self, t, y, dt, control):
        if callable(control):
            u_v, u_s = control(t, y)
        else:
            u_v, u_s = control

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
        d_r = self.L_f * (F_yf * np.cos(s_f) - F_res * np.sin(s_f)) / self.I_z - self.L_r * F_yr / self.I_z + n_s
        d_s = u_s
        d_x = v_x * np.cos(o) - v_y * np.sin(o)
        d_y = v_x * np.sin(o) + v_y * np.cos(o)
        d_o = r

        return [
            d_v_x, d_v_y, d_r,
            d_s, d_x, d_y, d_o
        ]
