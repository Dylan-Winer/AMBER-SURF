import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Generic Sensor 1 Data
ForceG1 = np.array([0, 0.31, 0.71, 1.41, 2.04])
VoutG1 = np.array([1.277, 1.287, 1.561, 1.887, 2.871])

# Generic Sensor 2 Data
ForceG2 = np.array([0, 0.32, 0.38])
VoutG2 = np.array([1.277, 2.177, 2.248])

# Function to compute linear fit and R^2
def linear_fit(x, y):
    a, b = np.polyfit(x, y, 1)
    y_pred = a * x + b
    r2 = 1 - np.sum((y - y_pred)**2) / np.sum((y - np.mean(y))**2)
    return a, b, r2

# Compute fits
aG1, bG1, r2_G1 = linear_fit(ForceG1, VoutG1)
aG2, bG2, r2_G2 = linear_fit(ForceG2, VoutG2)

# Generate fit lines
F_fit1 = np.linspace(min(ForceG1), max(ForceG1), 100)
V_fitG1 = aG1 * F_fit1 + bG1

F_fit2 = np.linspace(min(ForceG2), max(ForceG2), 100)
V_fitG2 = aG2 * F_fit2 + bG2

# Plot
plt.figure(figsize=(10, 6))
plt.scatter(ForceG1, VoutG1, color='purple', label='Square Pad FSR Data')
plt.plot(F_fit1, V_fitG1, color='purple', linestyle='--',
         label=f'Square Pad FSR Fit: V = {aG1:.4f}·F + {bG1:.4f}\nR² = {r2_G1:.4f}')

plt.scatter(ForceG2, VoutG2, color='brown', label='Thin Strip FSR Data')
plt.plot(F_fit2, V_fitG2, color='brown', linestyle='--',
         label=f'Thin Strip FSR Fit: V = {aG2:.4f}·F + {bG2:.4f}\nR² = {r2_G2:.4f}')

plt.xlabel('Force (N)')
plt.ylabel('Vout (V)')
plt.title('Linear Fit Comparison: Vout vs. Force (Cheap FSRs)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
