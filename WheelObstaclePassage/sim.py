#!env python3

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import scipy.integrate as sciint
import sys
import threading
import os
import time

import model
import importlib

done = False

import sim2

matplotlib.rc('font', family='Arial')

def reload():
    mod = model
    last = os.stat(mod.__file__).st_ctime

    while not done:
        if os.stat(mod.__file__).st_ctime != last:
            importlib.reload(mod)
            last = os.stat(mod.__file__).st_ctime
            init()
        time.sleep(0.5)

hotswap = threading.Thread(target=reload)
hotswap.start()


def init():
    global m, ir, data_x, data_y
    if len(sys.argv) > 1 and 'torque'.startswith(sys.argv[1]):
        m = model.model_torque()
    else:
        m = model.model_push()

    if type(m) == model.model_torque:
        m.init = s2i
    ir = sciint.ode(m.step).set_integrator('vode', atol=0.01)
    ir.set_initial_value(m.init, 0)

    data_x = []
    data_y = []

s2i,s2s = None,None
print('running sim2..',end='')
s2i, s2s = sim2.run()
print('OK')
print('init..',end='')
init()
print('OK')

print(m.Mf2(0.5, [0, 0, 5.5, 1.5]))
print(m.Mr2([0, 0, 5.5, 1.5]))

fig = plt.figure()
ax = fig.add_subplot(
    111, autoscale_on=False, aspect='equal', xlim=(0, 10), ylim=(0, 5))
ax.grid()

#fig.suptitle(type(m).__name__)

wheel = plt.Circle((0, 0), m.r_k, color='black', lw=2, fill=False)
ax.add_artist(wheel)
if type(m) == model.model_torque:
    wheel2 = plt.Circle((s2s[2], s2s[3]), m.r_k, color='black', lw=2, fill=False)
    ax.add_artist(wheel2)
ax.add_artist(plt.Line2D((0, m.x_h), (m.y_f, m.y_f), color='gray'))
ax.add_artist(plt.Line2D((m.x_h, m.x_h), (m.y_f, m.y_h), color='gray'))
ax.add_artist(plt.Line2D((m.x_h, 10), (m.y_h, m.y_h), color='gray'))

f_s = plt.Line2D((0, 0), (0, 0), color='red')
f_x = plt.Line2D((0, 0), (0, 0), color='green')
f_y = plt.Line2D((0, 0), (0, 0), color='blue')

ax.add_artist(f_s)
ax.add_artist(f_x)
ax.add_artist(f_y)
ax.set_xlabel(u'Расстояние (м)')
ax.set_ylabel(u'Расстояние (м)')

line, = ax.plot([], [], '-', color='black')
#text = ax.text(0.05, 0.05, '', transform=ax.transAxes)

data_x = []
data_y = []

dt = 10

def animate(i):
    state = ir.integrate(ir.t + dt/1000)
    data_x.append(state[2])
    data_y.append(state[3])
    line.set_xdata(data_x)
    line.set_ydata(data_y)
#    data_t.append(ir.t)
#    data_m.append(m.M)
    wheel.center = (state[2], state[3])
    if state[2] > 10:
        wheel.center = (m.init[2], m.init[3])
 #   text.set_text('v:({:.2f},{:.2f}), xy:({:.2f},{:.2f}) M:{:2f}'.format(*state, m.M))

    k_f = 0.05
    f_x.set_data((state[2], state[2] + m.f[0] * k_f), (state[3], state[3]))
    f_y.set_data((state[2], state[2]), (state[3], state[3] + m.f[1] * k_f))
    f_s.set_data((state[2], state[2] + m.f[0] * k_f),
                 (state[3], state[3] + m.f[1] * k_f))

    return (line, wheel, f_s, f_x, f_y)

ani = animation.FuncAnimation(fig, animate, interval=25, blit=True)
plt.tight_layout()
plt.show()

ani.save('out.gif', writer='imagemagick', fps=30)
done = True
hotswap.join()
print('fin')
