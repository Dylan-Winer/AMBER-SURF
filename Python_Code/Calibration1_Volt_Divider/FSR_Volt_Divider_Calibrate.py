import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Sample calibration data
R_fsr = np.array([13700, 7900, 5820, 4330])  # Resistance in Ohms
F = np.array([0.38, 0.76, 1.14, 1.52])       # Force in Newtons

# Power-law model: F = C * R^(-n)
def model(R, C, n):
    return C * R**(-n)

# Fit model to data
params, _ = curve_fit(model, R_fsr, F)
C, n = params
print(f"F ≈ {C:.2e} * R^-{n:.2f}")

# Generate fit curve
R_fit = np.linspace(min(R_fsr), max(R_fsr), 100)
F_fit = model(R_fit, *params)

# --- Plot 1: Log-Log Scale ---
plt.figure(figsize=(8, 5))
plt.loglog(R_fsr, F, 'o', label='Measured Data')
plt.loglog(R_fit, F_fit, '-', label=f'Fit: F = {C:.2e} * R^(-{n:.2f})')
plt.title('FSR Calibration (Interlink 406 Short) – Log-Log Scale')
plt.xlabel('Resistance (Ohms)')
plt.ylabel('Force (Newtons)')
plt.legend()
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.tight_layout()

# --- Plot 2: Linear Scale ---
plt.figure(figsize=(8, 5))
plt.plot(R_fsr, F, 'o', label='Measured Data')
plt.plot(R_fit, F_fit, '-', label=f'Fit: F = {C:.2e} * R^(-{n:.2f})')
plt.title('FSR Calibration (Interlink 406 Short) – Linear Scale')
plt.xlabel('Resistance (Ohms)')
plt.ylabel('Force (Newtons)')
plt.legend()
plt.grid(True, linestyle='--', linewidth=0.5)
plt.tight_layout()

plt.show()
