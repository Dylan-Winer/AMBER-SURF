import numpy as np
import matplotlib.pyplot as plt

# Old Sensor 2 Data
Force2_old = np.array([0, 4.59, 17.75, 28.54, 44.78, 102.32])
Vout2_old = np.array([1.277, 1.287, 1.342, 1.377, 1.442, 1.674])

# Redone Sensor 2 Data
Force2_new = np.array([0, 4.6, 11.07, 17.75, 44.78])
Vout2_new = np.array([1.277, 1.29, 1.313, 1.339, 1.439])

# Fit Old Sensor 2
a_old, b_old = np.polyfit(Force2_old, Vout2_old, 1)
Vout2_old_pred = a_old * Force2_old + b_old
r2_old = 1 - np.sum((Vout2_old - Vout2_old_pred)**2) / np.sum((Vout2_old - np.mean(Vout2_old))**2)

# Fit New Sensor 2
a_new, b_new = np.polyfit(Force2_new, Vout2_new, 1)
Vout2_new_pred = a_new * Force2_new + b_new
r2_new = 1 - np.sum((Vout2_new - Vout2_new_pred)**2) / np.sum((Vout2_new - np.mean(Vout2_new))**2)

# Print equations and R² values
print(f"Old Sensor 2: Vout ≈ {a_old:.4f} * Force + {b_old:.4f} (R² = {r2_old:.4f})")
print(f"New Sensor 2: Vout ≈ {a_new:.4f} * Force + {b_new:.4f} (R² = {r2_new:.4f})")

# Generate fits for plotting
F_fit_old = np.linspace(min(Force2_old), max(Force2_old), 100)
V_fit_old = a_old * F_fit_old + b_old

F_fit_new = np.linspace(min(Force2_new), max(Force2_new), 100)
V_fit_new = a_new * F_fit_new + b_new

# Plot both fits and data
plt.figure(figsize=(8, 5))
plt.scatter(Force2_old, Vout2_old, color='blue', label='Old Sensor 2 Data')
plt.plot(F_fit_old, V_fit_old, color='blue', linestyle='--',
         label=f'Old Fit: V = {a_old:.4f}·F + {b_old:.4f}\nR² = {r2_old:.4f}')

plt.scatter(Force2_new, Vout2_new, color='red', label='Redone Sensor 2 Data')
plt.plot(F_fit_new, V_fit_new, color='red', linestyle='--',
         label=f'Redone Fit: V = {a_new:.4f}·F + {b_new:.4f}\nR² = {r2_new:.4f}')

plt.xlabel('Force (N)')
plt.ylabel('Vout (V)')
plt.title('Comparison: Old vs Redone Sensor 2 Calibration')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# Compute percentage difference of coefficients from Redone vs Old Sensor 2
a_diff_percent = abs(a_old - a_new) / abs(a_old) * 100
b_diff_percent = abs(b_old - b_new) / abs(b_old) * 100

print(f"\n% Difference in Slope (a): {a_diff_percent:.2f}%")
print(f"% Difference in Intercept (b): {b_diff_percent:.2f}%")
