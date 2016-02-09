#!python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import scipy.integrate as sciint
import model

m = model.model_torque()

ir = sciint.ode(m.step).set_integrator('vode', atol=0.01)
ir.set_initial_value(m.init, 0)

plt.xkcd()

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, aspect='equal', xlim=(0, 10), ylim=(0, 5))
ax.grid()

wheel = plt.Circle((0,0), m.r_k, color='black', lw=2, fill=False)
ax.add_artist(wheel)
ax.add_artist(plt.Line2D((0, m.x_h), (m.y_f, m.y_f), color='gray'))
ax.add_artist(plt.Line2D((m.x_h, m.x_h), (m.y_f, m.y_h), color='gray'))
ax.add_artist(plt.Line2D((m.x_h, 10), (m.y_h, m.y_h), color='gray'))

f_s = plt.Line2D((0, 0), (0, 0), color='red')
f_x = plt.Line2D((0, 0), (0, 0), color='green')
f_y = plt.Line2D((0, 0), (0, 0), color='blue')

ax.add_artist(f_s)
ax.add_artist(f_x)
ax.add_artist(f_y)

line, = ax.plot([], [], '-', color='black')
text = ax.text(0.05, 0.05, '', transform=ax.transAxes)

data_x = []
data_y = []

dt = 25

def animate(i):
	state = ir.integrate(ir.t + dt/1000)
	data_x.append(state[2])
	data_y.append(state[3])
	line.set_xdata(data_x)
	line.set_ydata(data_y)
	wheel.center = (state[2], state[3])
	text.set_text('v:({:.2f},{:.2f}), xy:({:.2f},{:.2f})'.format(*state))
	
	k_f = 0.05
	f_x.set_data((state[2], state[2] + m.f[0] * k_f), (state[3], state[3]))
	f_y.set_data((state[2], state[2]), (state[3], state[3] + m.f[1] * k_f))
	f_s.set_data((state[2], state[2] + m.f[0] * k_f), (state[3], state[3] + m.f[1] * k_f))
	
	return (line,wheel,text, f_s, f_x, f_y)

ani = animation.FuncAnimation(fig, animate, interval=25, blit=True)

plt.show()