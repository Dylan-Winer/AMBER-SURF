import numpy as np
import matplotlib.pyplot as plt

# === New sensor data (from attached sheet) ===
# Each: Force (N) vs Vout (V)

Force_FSR1 = np.array([0.00, 17.75, 47.87])
Vout_FSR1  = np.array([1.342, 1.397, 1.497])

Force_FSR2 = np.array([0.00, 17.75, 47.87])
Vout_FSR2  = np.array([1.342, 1.416, 1.532])

Force_FSR3 = np.array([0.00, 17.75, 47.87])
Vout_FSR3  = np.array([1.348, 1.429, 1.555])

Force_FSR4 = np.array([0.00, 17.75, 47.87])
Vout_FSR4  = np.array([1.345, 1.426, 1.548])

# Linear fit function (Vout = a * Force + b)
def linear_fit(x, y):
    a, b = np.polyfit(x, y, 1)
    y_pred = a * x + b
    r2 = 1 - np.sum((y - y_pred)**2) / np.sum((y - np.mean(y))**2)
    return a, b, r2

# Fits
fits = {
    'FSR1': linear_fit(Force_FSR1, Vout_FSR1),
    'FSR2': linear_fit(Force_FSR2, Vout_FSR2),
    'FSR3': linear_fit(Force_FSR3, Vout_FSR3),
    'FSR4': linear_fit(Force_FSR4, Vout_FSR4),
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
