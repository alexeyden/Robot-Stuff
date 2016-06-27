#!env python3

import matplotlib
import matplotlib.pyplot as plt

import scipy.integrate as sciint
import sys
from math import cos,asin

import model

matplotlib.rc('font', family='Arial')

def run():
    data_x = []
    data_y = []

    dt = 0.1
    tmax = 10
    state = None

    m = model.model_push()
    ir = sciint.ode(m.step).set_integrator('vode', atol=0.01)
    ir.set_initial_value(m.init, 0)

    while state is None or state[2] < m.x_h and ir.t < tmax:
        state = ir.integrate(ir.t + dt/1000)
        data_x.append(ir.t)
        data_y.append(m.M)

    if ir.t >= tmax:
        print('front failed')
        plt.ylim(0, m.Mmax + 10)
        plt.plot(data_x, data_y)
        plt.xlabel('Время t [с]')
        plt.ylabel('Момент М(t) [Н м]')
        plt.show()
        return None

    t1 = ir.t
    m = model.model_torque()
    ir = sciint.ode(m.step).set_integrator('vode', atol=0.01)
    m.init = [
        state[0], 0,
        state[2] - m.L * cos(asin(((state[3] - m.y_f)/m.L))), m.r_k + m.y_f
    ]
    state2 = state[:]
    ir.set_initial_value(m.init, 0)
    state = None
    while state is None or state[2] < m.x_h and ir.t + t1 < tmax:
        state = ir.integrate(ir.t + dt/1000)
        data_x.append(ir.t + t1)
        data_y.append(m.M)

    if ir.t + t1 >= tmax:
        print('rear failed')
        return None
    print('all pass')
    plt.ylim(0, m.Mmax + 10)
    plt.plot(data_x, data_y)
    plt.xlabel('Время t [с]')
    plt.ylabel('Момент М(t) [Н м]')
    plt.show()
    return (m.init, state2)

if __name__ == "__main__":
    run()
