import numpy as np
import matplotlib.pyplot as plt

# Create a grid of x and y values
N = 100
x_vals = np.linspace(-5, 5, N)
y_vals = np.linspace(-5, 5, N)
X, Y = np.meshgrid(x_vals, y_vals)

# Compute z-values for arctan(y/x)
Z_arctan = np.full_like(X, np.nan, dtype=float)
mask = (X != 0)  # Avoid division by zero
Z_arctan[mask] = np.arctan(Y[mask] / X[mask])

# Compute z-values for atan2(y, x)
Z_arctan2 = np.arctan2(Y, X)

# Plotting both surfaces on the same plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot arctan(y/x) surface
surf1 = ax.plot_surface(X, Y, Z_arctan, cmap='viridis', alpha=0.7, edgecolor='none')

# Plot atan2(y, x) surface
surf2 = ax.plot_surface(X, Y, Z_arctan2, cmap='plasma', alpha=0.7, edgecolor='none')

ax.set_title('Comparison of arctan(y/x) and atan2(y, x)')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('Angle (radians)')

plt.show()
