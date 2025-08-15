import numpy as np
import matplotlib.pyplot as plt

# Sensor Data from Spreadsheet
Force_A3  = np.array([0, 17.74, 28.81, 39.53, 47.87])
Vout_A3   = np.array([1.355, 1.410, 1.448, 1.484, 1.516])

Force_A0  = np.array([0, 17.75, 28.81, 47.88])
Vout_A0   = np.array([1.355, 1.439, 1.484, 1.561])

Force_A6  = np.array([0, 17.75, 47.88])
Vout_A6   = np.array([1.352, 1.419, 1.548])

Force_A9  = np.array([0, 17.75, 47.88])
Vout_A9   = np.array([1.355, 1.416, 1.516])

Force_A10 = np.array([0, 17.75, 48.87])
Vout_A10  = np.array([1.348, 1.377, 1.429])

Force_A11 = np.array([0, 17.75, 47.87])
Vout_A11  = np.array([1.352, 1.381, 1.429])

Force_A12 = np.array([0, 17.75, 48.87])
Vout_A12  = np.array([1.355, 1.400, 1.468])

# Linear fit function
def linear_fit(x, y):
    a, b = np.polyfit(x, y, 1)
    y_pred = a * x + b
    r2 = 1 - np.sum((y - y_pred)**2) / np.sum((y - np.mean(y))**2)
    return a, b, r2

# Fit calculations
a3,  b3,  r2_3  = linear_fit(Force_A3,  Vout_A3)
a0,  b0,  r2_0  = linear_fit(Force_A0,  Vout_A0)
a6,  b6,  r2_6  = linear_fit(Force_A6,  Vout_A6)
a9,  b9,  r2_9  = linear_fit(Force_A9,  Vout_A9)
a10, b10, r2_10 = linear_fit(Force_A10, Vout_A10)
a11, b11, r2_11 = linear_fit(Force_A11, Vout_A11)
a12, b12, r2_12 = linear_fit(Force_A12, Vout_A12)

# Plotting
F_fit = np.linspace(0, 55, 100)
plt.figure(figsize=(10, 6))

plt.plot(F_fit, a0 * F_fit + b0,  color='green',  linestyle='--', label=f'A0 Fit (R²={r2_0:.4f})')
plt.scatter(Force_A0, Vout_A0,   color='green',  marker='s')

plt.plot(F_fit, a3 * F_fit + b3,  color='blue',   linestyle='--', label=f'A3 Fit (R²={r2_3:.4f})')
plt.scatter(Force_A3, Vout_A3,   color='blue',   marker='o')

plt.plot(F_fit, a6 * F_fit + b6,  color='orange', linestyle='--', label=f'A6 Fit (R²={r2_6:.4f})')
plt.scatter(Force_A6, Vout_A6,   color='orange', marker='^')

plt.plot(F_fit, a9 * F_fit + b9,  color='red',    linestyle='--', label=f'A9 Fit (R²={r2_9:.4f})')
plt.scatter(Force_A9, Vout_A9,   color='red',    marker='v')

plt.plot(F_fit, a10 * F_fit + b10, color='purple', linestyle='--', label=f'A10 Fit (R²={r2_10:.4f})')
plt.scatter(Force_A10, Vout_A10,  color='purple', marker='P')

plt.plot(F_fit, a11 * F_fit + b11, color='brown',  linestyle='--', label=f'A11 Fit (R²={r2_11:.4f})')
plt.scatter(Force_A11, Vout_A11,  color='brown',  marker='X')

plt.plot(F_fit, a12 * F_fit + b12, color='teal',   linestyle='--', label=f'A12 Fit (R²={r2_12:.4f})')
plt.scatter(Force_A12, Vout_A12,  color='teal',   marker='D')

plt.xlabel('Force (N)')
plt.ylabel('Vout (V)')
plt.title('Linear Regression Modeling: Vout vs. Force (FSRs)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
