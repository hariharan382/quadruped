import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

x = np.linspace(0, 2 * np.pi, 100)

# generate 10 curves
y = np.sin(x.reshape(-1, 1) + np.random.uniform(0, 2 * np.pi, (1, 10)))

fig, ax = plt.subplots()
ax.set(xlim=(0, 2 * np.pi), ylim=(-1.5, 1.5))
# lines = [ax.plot([], [], lw=2)[0] for _ in range(y.shape[1])]
lines = ax.plot(np.empty((0, y.shape[1])), np.empty((0, y.shape[1])), lw=2)

def animate(i):
    for line_k, y_k in zip(lines, y.T):
        line_k.set_data(x[:i], y_k[:i])
    return lines

anim = FuncAnimation(fig, animate, frames=x.size, interval=200, repeat=False)
plt.show()