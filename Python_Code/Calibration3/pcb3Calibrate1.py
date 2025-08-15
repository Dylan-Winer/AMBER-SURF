import numpy as np
import matplotlib.pyplot as plt

# =========================
# Data from the attached image
# Columns shown there: Force (N), Raw, Vout (V)
# =========================
data = {
    "FSR1": {
        "Force": np.array([0.00, 17.75, 47.87]),
        "Vout":  np.array([1.352, 1.435, 1.594]),
    },
    "FSR2": {
        "Force": np.array([0.00, 17.75, 47.87]),
        "Vout":  np.array([1.358, 1.426, 1.552]),
    },
    "FSR3": {
        "Force": np.array([0.00, 17.75, 47.87]),
        "Vout":  np.array([1.358, 1.426, 1.545]),
    },
    "FSR4": {
        "Force": np.array([0.00, 17.75, 47.87]),
        "Vout":  np.array([1.352, 1.413, 1.526]),
    },
}

def linear_fit(x, y):
    """Fit y = a*x + b. Return a, b, R^2."""
    a, b = np.polyfit(x, y, 1)
    yhat = a * x + b
    ss_res = np.sum((y - yhat)**2)
    ss_tot = np.sum((y - np.mean(y))**2)
    r2 = 1 - ss_res/ss_tot if ss_tot != 0 else 1.0
    return a, b, r2

# Compute fits and print inverse calibration equations
fits = {}
print("Calibration Equations (Force from Vout):")
for name, d in data.items():
    a, b, r2 = linear_fit(d["Force"], d["Vout"])
    fits[name] = (a, b, r2)
    inv_slope = 1.0 / a
    inv_offset = -b / a
    print(f"  {name}: Force ≈ {inv_slope:.3f}·Vout + {inv_offset:.3f}   (R² = {r2:.4f})")

# Plot data + fitted lines
F_fit = np.linspace(0, 55, 200)
plt.figure(figsize=(9, 6))

for name, d in data.items():
    a, b, r2 = fits[name]
    plt.scatter(d["Force"], d["Vout"], label=f"{name} data")
    plt.plot(F_fit, a*F_fit + b, linestyle="--", label=f"{name} fit (R²={r2:.4f})")

plt.title("FSR Calibration: Vout vs Force with Linear Fits")
plt.xlabel("Force (N)")
plt.ylabel("Vout (V)")
plt.grid(True, alpha=0.3)
plt.legend(ncol=2)
plt.tight_layout()
plt.show()
