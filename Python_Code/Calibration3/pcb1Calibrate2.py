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

# Fit calculations (Vout = a * Force + b)
fits = {
    'A3':  linear_fit(Force_A3, Vout_A3),
    'A0':  linear_fit(Force_A0, Vout_A0),
    'A6':  linear_fit(Force_A6, Vout_A6),
    'A9':  linear_fit(Force_A9, Vout_A9),
    'A10': linear_fit(Force_A10, Vout_A10),
    'A11': linear_fit(Force_A11, Vout_A11),
    'A12': linear_fit(Force_A12, Vout_A12)
}

# Print calibration equations: Force = (Vout - b) / a
print("Calibration Equations (Force in N from Vout):")
for name, (a, b, r2) in fits.items():
    if abs(a) < 1e-6:
        print(f"{name}: Undefined slope (a ≈ 0)")
    else:
        slope_inv = 1 / a
        offset = -b / a
        print(f"{name}: Force ≈ ({slope_inv:.3f}) × Vout + ({offset:.3f})  (R² = {r2:.4f})")
