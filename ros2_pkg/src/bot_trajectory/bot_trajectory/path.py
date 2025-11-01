import numpy as np
import matplotlib.pyplot as plt
import pandas as pd  # for saving CSV

# Natural Cubic Spline Interpolation

# Given data points
x = np.array([0, 1, 2, 3, 4, 5], dtype=float)
y = np.array([0, 0.8, 0.9, 0.1, -0.8, -1.0], dtype=float)
n = len(x)

# Step 1: Compute interval widths
h = np.diff(x)

# Step 2: Build RHS vector α
alpha = np.zeros(n)
for i in range(1, n - 1):
    alpha[i] = 6 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])

# Step 3: Build tridiagonal matrix A (natural boundary conditions)
A = np.zeros((n, n))
A[0, 0] = 1
A[-1, -1] = 1
for i in range(1, n - 1):
    A[i, i - 1] = h[i - 1]
    A[i, i] = 2 * (h[i - 1] + h[i])
    A[i, i + 1] = h[i]

# Step 4: Solve for M (second derivatives)
M = np.linalg.solve(A, alpha)

# Step 5: Compute spline coefficients a, b, c, d
a = np.zeros(n - 1)
b = np.zeros(n - 1)
c = np.zeros(n - 1)
d = np.zeros(n - 1)
for i in range(n - 1):
    a[i] = y[i]
    b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] / 6) * (2 * M[i] + M[i + 1])
    c[i] = M[i] / 2
    d[i] = (M[i + 1] - M[i]) / (6 * h[i])

# Step 6: Evaluate spline and its derivative (dy/dx)
X = np.linspace(x[0], x[-1], 500)
S = np.zeros_like(X)
S_dot = np.zeros_like(X)

for i in range(n - 1):
    idx = (X >= x[i]) & (X <= x[i + 1])
    dx = X[idx] - x[i]
    S[idx] = a[i] + b[i]*dx + c[i]*dx**2 + d[i]*dx**3
    S_dot[idx] = b[i] + 2*c[i]*dx + 3*d[i]*dx**2

v = 0.2
distance = np.sqrt(np.diff(X)**2 + np.diff(S)**2)
t = np.zeros_like(X)
t[1:] = np.cumsum(distance / v)

# Save x, y, t
data = pd.DataFrame({'x': X, 'y': S, 't': t})
data.to_csv('spline_xy_time.csv', index=False)
print("Saved spline data to 'spline_xy_points.csv'")

# === Plot Trajectory ===
plt.figure(figsize=(6, 5))
plt.plot(x, y, 'o', label='Data Points')
plt.plot(X, S, '-', label='Spline Path')
plt.title("Trajectory in XY-plane")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
