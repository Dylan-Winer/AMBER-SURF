import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Sensor 1 Data
Force1 = np.array([0, 4.8, 7.76, 10.52, 17.75, 22.33, 33.4, 44.77, 53.5, 102.32])
Vout1 = np.array([1.277, 1.29, 1.30, 1.31, 1.335, 1.348, 1.39, 1.426, 1.452, 1.623])

# Sensor 2 Data
Force2 = np.array([0, 4.59, 17.75, 28.54, 44.78, 102.32])
Vout2 = np.array([1.277, 1.287, 1.342, 1.377, 1.442, 1.674])

# Sensor 3 Data
Force3 = np.array([0, 4.59, 11.07, 17.76, 33.14])
Vout3 = np.array([1.284, 1.3, 1.329, 1.352, 1.419])

# Sensor 4 Data
Force4 = np.array([0, 11.07, 33.14])
Vout4 = np.array([1.287, 1.332, 1.432])

# Function to compute linear fit and R^2
def linear_fit(x, y):
    a, b = np.polyfit(x, y, 1)
    y_pred = a * x + b
    r2 = 1 - np.sum((y - y_pred)**2) / np.sum((y - np.mean(y))**2)
    return a, b, r2

# Compute fits
a1, b1, r2_1 = linear_fit(Force1, Vout1)
a2, b2, r2_2 = linear_fit(Force2, Vout2)
a3, b3, r2_3 = linear_fit(Force3, Vout3)
a4, b4, r2_4 = linear_fit(Force4, Vout4)

# Print equations and R² values
print(f"Sensor 1: Vout ≈ {a1:.4f} * Force + {b1:.4f} (R² = {r2_1:.4f})")
print(f"Sensor 2: Vout ≈ {a2:.4f} * Force + {b2:.4f} (R² = {r2_2:.4f})")
print(f"Sensor 3: Vout ≈ {a3:.4f} * Force + {b3:.4f} (R² = {r2_3:.4f})")
print(f"Sensor 4: Vout ≈ {a4:.4f} * Force + {b4:.4f} (R² = {r2_4:.4f})")

# Generate fit lines
F_fit = np.linspace(0, max(Force1), 100)
V_fit1 = a1 * F_fit + b1
V_fit2 = a2 * F_fit + b2
V_fit3 = a3 * F_fit + b3
V_fit4 = a4 * F_fit + b4

# Plot
plt.figure(figsize=(10, 6))
plt.scatter(Force1, Vout1, color='blue', label='FSR 1 Data')
plt.plot(F_fit, V_fit1, color='blue', linestyle='--', label=f'FSR 1 Fit (R²={r2_1:.4f})')

plt.scatter(Force2, Vout2, color='green', label='FSR 2 Data')
plt.plot(F_fit, V_fit2, color='green', linestyle='--', label=f'FSR 2 Fit (R²={r2_2:.4f})')

plt.scatter(Force3, Vout3, color='orange', label='FSR 3 Data')
plt.plot(F_fit, V_fit3, color='orange', linestyle='--', label=f'FSR 3 Fit (R²={r2_3:.4f})')

plt.scatter(Force4, Vout4, color='red', label='FSR 4 Data')
plt.plot(F_fit, V_fit4, color='red', linestyle='--', label=f'FSR 4 Fit (R²={r2_4:.4f})')

plt.xlabel('Force (N)')
plt.ylabel('Vout (V)')
plt.title('Linear Fit Comparison: Vout vs. Force (FSRs 1–4)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

# Compute percentage differences relative to Sensor 1
def percent_diff(base, compare):
    return abs(base - compare) / abs(base) * 100

a_diff = {
    'FSR 2': percent_diff(a1, a2),
    'FSR 3': percent_diff(a1, a3),
    'FSR 4': percent_diff(a1, a4)
}

b_diff = {
    'FSR 2': percent_diff(b1, b2),
    'FSR 3': percent_diff(b1, b3),
    'FSR 4': percent_diff(b1, b4)
}

# Add FSR 1 as reference with 0% difference
a_diff["FSR 1"] = 0.0
b_diff["FSR 1"] = 0.0

# Create a DataFrame
diff_df = pd.DataFrame({
    "Δ Slope (%)": a_diff,
    "Δ Intercept (%)": b_diff
}).sort_index()
print(diff_df)

# Export to CSV
csv_path = "C:/Users/dywin/Documents/Caltech/Other_Code/Calibration2/fsr_fit_differences.csv"
diff_df.to_csv(csv_path)

diff_df, csv_path
