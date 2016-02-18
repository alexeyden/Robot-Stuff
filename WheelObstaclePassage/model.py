from math import sin, cos, tan, atan, atan2, sqrt
import numpy as np


class model:
    g = 9.81

    m_p = 100  # Масса подрессорной части
    r_k = 0.5  # Радиус колеса

    y_f = 1
    x_h = 6
    y_h = 1.4

    M = 0.4e3

    Mmax = 0.8e3
    Vmax = 5

    k1, k2 = 30000, 2000

    init = [0, 0, 0.5, 1.4]

    cr1 = k1
    cr2 = k1
    phi2 = 1

    def __init__(self):
        self.f = [0, 0]
        self.M = 0

    def Mf1(self, r, y):
        dx1, dz1, x1, z1 = y
        cr1 = model.cr1
        cr2 = model.cr2
        phi2 = model.phi2

        a = atan2(model.x_h - x1, z1 - model.y_h)
        G = model.m_p / 2 * model.g

        P = (G - cr1 *
             (model.r_k - model.r_k * cos(a) - (model.y_h - model.y_f))) * \
            cr2 * (tan(a) - phi2)/(cr2 * (1 + phi2 * tan(a)) + cr1)

        return P * r

    def Mf2(self, r, y):
        dx1, dz1, x1, z1 = y
        a = atan2(model.x_h - x1, z1 - model.y_h)
        nu = atan(a - model.phi2)

        G = model.m_p / 2 * model.g
        P = G * tan(a - nu)
        return P * r

    def Mr1(self, y):
        dx1, dz1, x1, z1 = y
        G = model.m_p / 2 * model.g
        a = atan2(model.x_h - x1, z1 - model.y_h)
        cr1 = model.cr1
        cr2 = model.cr2
        phi2 = model.ph2

        return (
                G - cr1 *
                (model.r_k - model.r_k * cos(a) - (model.y_h - model.y_f))) * \
            cr2 * phi2 / ((cr2 * (1 + phi2 * tan(a)) + cr1) * cos(a)) * \
            (model.r_k - (G - cr1 * (model.r_k - model.r_k *
                                     cos(a) - (model.y_h - model.y_f)) *
                          (1/((cr2 * (1 + phi2 * tan(a)) + cr1) * cos(a)))))

    def Mr2(self, y):
        dx1, dz1, x1, z1 = y
        G = model.m_p / 2 * model.g
        a = atan2(model.x_h - x1, z1 - model.y_h)
        cr2 = model.cr2
        nu = atan(a - model.phi2)

        return G/model.cr2 * sin(nu)/cos(a - nu) * (
            cr2 * model.r_k - G * cos(nu)/cos(a-nu))

    def f_R(self, t, y):
        pass

    def f_N(self, t, y, level):
        pass

    def step(self, t, y):
        pass


class model_push(model):

    def __init__(self):
        super().__init__()

    def f_R(self, t, y):
        dx1, dz1, x1, z1 = y

        a = atan2(model.x_h - x1, z1 - model.y_h)

        r = np.sqrt((model.x_h - x1) ** 2 + (z1 - model.y_h) ** 2)
        r_diff = (-2 * (model.x_h - x1) * dx1 + 2 * (-model.y_h + z1) * dz1) / (2 * sqrt((model.x_h - x1) ** 2 + (-model.y_h + z1) ** 2))  # dr/dt

        R = model.k1 * (model.r_k - r) - model.k2 * r_diff

        F_m = model.M/model.r_k * cos(a)  # - self.m_p * self.g * sin(a)

        F_mx = - R * sin(a) + F_m * cos(a)
        F_my = R * cos(a) + F_m * sin(a)

        return (F_mx, F_my)

    def f_N(self, t, y, level):
        dx1, dz1, x1, z1 = y

        r = level - (z1 - model.r_k)
        r_diff = -dz1
        R = model.k1 * r + model.k2 * r_diff

        F_mx = 0
        F_my = R

        return (F_mx, F_my)

    def step(self, t, y):
        dx1, dz1, x1, z1 = y

        F_mx = 0
        F_my = 0

        G = model.m_p * model.g

        if z1 - model.r_k <= model.y_f and x1 < model.x_h:
            F_mx, F_my = self.f_N(t, y, model.y_f)
            self.M = model.Mmax if dx1 < model.Vmax else 0

        if z1 - model.r_k <= model.y_h and x1 >= model.x_h:
            F_mx, F_my = self.f_N(t, y, model.y_h)
            self.M = model.Mmax if dx1 < model.Vmax else 0

        F_mx += model.M/model.r_k

        # преодоление препятствия
        if np.linalg.norm([x1 - model.x_h, z1 - model.y_h]
                          ) - model.r_k < 0.01 and x1 < model.x_h:
            fx, fy = self.f_R(t, y)
            F_mx = fx
            F_my += fy
            self.M = self.Mf1(model.r_k, y) if z1 - model.r_k <= model.y_f else self.Mf2(model.r_k, y)

        F_sx = F_mx
        F_sy = F_my - G

        eq_ddx1 = F_sx / model.m_p
        eq_ddz1 = F_sy / model.m_p
        eq_dx1 = dx1
        eq_dz1 = dz1

        self.f = [eq_ddx1, eq_ddz1]

        return [
                eq_ddx1, eq_ddz1,
                eq_dx1, eq_dz1
            ]


class model_torque(model):

    def __init__(self):
        super().__init__()

    def f_R(self, t, y):
        dx1, dz1, x1, z1 = y

        a = atan2(model.x_h - x1, z1 - model.y_h)

        r = np.sqrt((model.x_h - x1)**2 + (z1 - model.y_h)
                    ** 2)  # радиус деформации
        r_diff = (-2 * (model.x_h - x1) * dx1 + 2 * (-model.y_h + z1) * dz1) / (2 *
                                                                                sqrt((model.x_h - x1) ** 2 + (-model.y_h + z1) ** 2))  # dr/dt

        R = model.k1 * (model.r_k - r) - model.k2 * r_diff

        F_m = model.M/r - self.m_p * self.g * sin(a)

        F_mx = - R * sin(a) + F_m * cos(a)
        F_my = R * cos(a) + F_m * sin(a)

        return (F_mx, F_my)

    def f_N(self, t, y, level):
        dx1, dz1, x1, z1 = y

        r = level - (z1 - model.r_k)
        r_diff = -dz1
        R = model.k1 * r + model.k2 * r_diff

        F_mx = model.M/model.r_k
        F_my = R

        return (F_mx, F_my)

    def step(self, t, y):
        dx1, dz1, x1, z1 = y

        F_mx = 0
        F_my = 0

        G = model.m_p * model.g

        if z1 - model.r_k <= model.y_f and x1 < model.x_h:
            F_mx, F_my = self.f_N(t, y, model.y_f)
            self.M = model.Mmax if dx1 < model.Vmax else 0

        if z1 - model.r_k <= model.y_h and x1 >= model.x_h:
            F_mx, F_my = self.f_N(t, y, model.y_h)
            self.M = model.Mmax if dx1 < model.Vmax else 0

        # преодоление препятствия
        if np.linalg.norm([x1 - model.x_h, z1 - model.y_h]
                          ) - model.r_k < 0.01 and x1 < model.x_h:
            fx, fy = self.f_R(t, y)
            F_mx += fx
            F_my += fy
            self.M = self.Mr1(y) if z1 - model.r_k <= model.y_f else self.Mr2(y)

        F_sx = F_mx
        F_sy = F_my - G

        eq_ddx1 = F_sx / model.m_p
        eq_ddz1 = F_sy / model.m_p
        eq_dx1 = dx1
        eq_dz1 = dz1

        self.f = [eq_ddx1, eq_ddz1]

        return [
                eq_ddx1, eq_ddz1,
                eq_dx1, eq_dz1
        ]
