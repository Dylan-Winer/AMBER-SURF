def compute_cop_grf(force1, force2, pos1=(0, 1), pos2=(0, 5)):
    F1, F2 = force1, force2
    x1, y1 = pos1
    x2, y2 = pos2
    
    F_total = F1 + F2
    if F_total == 0:
        return None  # No contact
    
    COP_x = (F1 * x1 + F2 * x2) / F_total
    COP_y = (F1 * y1 + F2 * y2) / F_total
    
    return {
        "COP": (COP_x, COP_y),
        "F_total": F_total
    }


import matplotlib.pyplot as plt

cop_list = []
F_total_list = []

for i, row in df.iterrows():
    result = compute_cop_grf(row["Force1_N"], row["Force2_N"])
    if result:
        cop_list.append(result["COP"][1])  # Y only
        F_total_list.append(result["F_total"])
    else:
        cop_list.append(None)
        F_total_list.append(0)

plt.figure(figsize=(10, 4))
plt.plot(df["Time_s"], cop_list, label="COP_y (inches)")
plt.plot(df["Time_s"], F_total_list, label="Total Force (N)", alpha=0.5)
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("COP Y / Total Force")
plt.title("Center of Pressure and Total GRF Over Time")
plt.grid(True)
plt.tight_layout()
plt.show()
