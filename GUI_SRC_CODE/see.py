import matplotlib.pyplot as plt
import numpy as np

r2d = 180/np.pi
fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(1,1,1)
x = np.arange(-1,1.,0.01)
y = np.arange(1,-1.,-0.01)

ycirc = np.sqrt(1 - x**2)
x1 = -1
y1 = -1
print('atan2 for x = -1, y = -1',np.arctan2(y1,x1)*r2d)
print('atan,for x = -1, y = -1',np.arctan(y1/x1)*r2d)

ax.plot(np.zeros(y.shape),y)
ax.plot(x, np.zeros(x.shape))
ax.plot(x, ycirc,'black')
ax.plot(x, -ycirc,'black')
ax.plot([0, -1], [0,-1],color='blue')
ax.plot([0, 1], [0,1],color='red')
ax.plot(1,1,marker='o',color='red')
ax.plot(-1,-1,marker='o',color='blue')

ax.set_aspect('equal')
ax.set_ylim([-1.2, 1.2])
ax.set_xlim([-1.4, 1.2])
ax.text(1,0,'0')
ax.text(0,1.1,'$\pi/2, (90)$')
ax.text(-1.19,0,'$\pm\pi$')
ax.text(-1.39,-0.2,'($\pm180$)')
ax.text(0,-1.1,'$-\pi/2, (-90)$')
ax.text(0.5,0.5,'East')
ax.text(-0.5,0.5,'East')
ax.text(0.5,-0.5,'West')
ax.text(-0.5,-0.5,'West')
ax.set_xlabel('x coordinate')
ax.set_ylabel('y coordinate')
ax.set_title('(x,y) at the equator')
plt.show()