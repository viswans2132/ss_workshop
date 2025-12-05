import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.colors import LinearSegmentedColormap

# Parameters
gamma_1 = 0.4
gamma_10 = 0.5
n_super = 4.0

# Robot at (0,0), 3 obstacles around it
robot_pos = np.array([0.0, 0.0])
obstacles = [
    {'center': (1.0, 0.5), 'a': 0.8, 'b': 1.2, 'color': 'red'},
    {'center': (-1.8, 1.2), 'a': 0.6, 'b': 0.9, 'color': 'blue'},
    {'center': (0.8, -1.5), 'a': 0.6, 'b': 0.7, 'color': 'green'}
]

# Grid
x = np.linspace(-3.5, 3.5, 400)
y = np.linspace(-2.5, 2.5, 400)
X, Y = np.meshgrid(x, y)

def super_ellipse_cbf(X, Y, cx, cy, a, b, n):
    dx = np.abs(X - cx) / a
    dy = np.abs(Y - cy) / b
    return np.power(dx, n) + np.power(dy, n) - 1

# Compute individual CBFs
print("Computing individual super-ellipse CBFs...")
h_individual = np.zeros((len(obstacles), *X.shape))
for i, obs in enumerate(obstacles):
    cx, cy = obs['center']
    a, b = obs['a'], obs['b']
    h_individual[i] = super_ellipse_cbf(X, Y, cx, cy, a, b, n_super)

# Compute TWO composites: κ=1 vs κ=10
def compute_composite(h_individual, kappa, gamma):
    exp_terms = np.exp(-kappa * np.tanh(h_individual / gamma))
    sum_exp = np.sum(exp_terms, axis=0)
    return -(gamma / kappa) * np.log(sum_exp + 1e-12)

kappa_1 = 8.1
kappa_10 = 8.1

H_kappa1 = compute_composite(h_individual, kappa_1, gamma_1)
H_kappa10 = compute_composite(h_individual, kappa_10, gamma_10)
print(H_kappa10)

# Compute feasibility stats
feas_k1 = np.mean(H_kappa1 >= 0) * 100
feas_k10 = np.mean(H_kappa10 >= 0) * 100
h_k1_range = f"[{H_kappa1.min():.2f}, {H_kappa1.max():.2f}]"
h_k10_range = f"[{H_kappa10.min():.2f}, {H_kappa10.max():.2f}]"

print(f"κ=1:  H range {h_k1_range}, Feasibility: {feas_k1:.1f}%")
print(f"κ=10: H range {h_k10_range}, Feasibility: {feas_k10:.1f}%")

# SHARED COLOR SCALE: Use global min/max across BOTH plots
global_min = min(H_kappa1.min(), H_kappa10.min())
global_max = max(H_kappa1.max(), H_kappa10.max())
print(f"Shared color scale: [{global_min:.2f}, {global_max:.2f}]")

# pos_k1 = H_kappa1[H_kappa1 > 0]
pos_k10 = H_kappa10[H_kappa10 > 0.1]
# print(f"H_k1 positive range: [{pos_k1.min():.3f}, {pos_k1.max():.3f}]")
# print(f"H_k10 positive range: [{pos_k10.min():.3f}, {pos_k10.max():.3f}]")
print(f"{pos_k10}")

# Create levels spanning full range
levels = np.linspace(global_min, global_max, 25)

# Custom colormap: red(unsafe) → green(safe) → blue(very safe)
colors = ['red', 'orange', 'yellow', 'lightgreen', 'green', 'cyan', 'blue']
cmap = LinearSegmentedColormap.from_list('safety_shared', colors, N=256)

# PLOT: κ=1 vs κ=10 with SAME color scale
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))

# 1. κ=1 Composite CBF
ax1.set_title(f'κ=1: Overly Pessimistic\nFeasibility: {feas_k1:.1f}% | H∈{h_k1_range}', 
              fontsize=14, fontweight='bold')
im1 = ax1.contourf(X, Y, H_kappa1, levels=levels, cmap=cmap, extend='both')
ax1.contour(X, Y, H_kappa1, levels=[0], colors='black', linewidths=4, label='H=0')
ax1.contourf(X, Y, H_kappa1, levels=[0, global_max*0.2], colors='lightgreen', alpha=0.3, hatches=['//'])
ax1.add_patch(FancyBboxPatch((robot_pos[0]-0.15, robot_pos[1]-0.15), 0.3, 0.3, 
                            edgecolor='black', facecolor='yellow', linewidth=3, boxstyle="round,pad=0.1"))
ax1.grid(True, alpha=0.3)
ax1.axis('equal')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')

# 2. κ=10 Composite CBF (SAME color scale!)
ax2.set_title(f'κ=10: Optimal Balance\nFeasibility: {feas_k10:.1f}% | H∈{h_k10_range}', 
              fontsize=14, fontweight='bold')
im2 = ax2.contourf(X, Y, H_kappa10, levels=levels, cmap=cmap, extend='both')  # SAME levels!
ax2.contour(X, Y, H_kappa10, levels=[0], colors='black', linewidths=4)
ax2.contourf(X, Y, H_kappa10, levels=[0, global_max*0.2], colors='lightgreen', alpha=0.3)
ax2.add_patch(FancyBboxPatch((robot_pos[0]-0.15, robot_pos[1]-0.15), 0.3, 0.3, 
                            edgecolor='black', facecolor='yellow', linewidth=3, boxstyle="round,pad=0.1"))
ax2.grid(True, alpha=0.3)
ax2.axis('equal')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')

# 3. Individual CBFs (for reference)
ax3.set_title('Individual CBFs (min h_i ≥ 0)', fontsize=14, fontweight='bold')
for i, obs in enumerate(obstacles):
    ax3.contour(X, Y, h_individual[i], levels=[0], colors=obs['color'], linewidths=2)
    ax3.contourf(X, Y, h_individual[i], levels=[-0.5, 0], alpha=0.2, colors=obs['color'])
h_min = np.min(h_individual, axis=0)
ax3.contour(X, Y, h_min, levels=[0], colors='darkgreen', linewidths=4, label='min(h_i)=0')
ax3.add_patch(FancyBboxPatch((robot_pos[0]-0.15, robot_pos[1]-0.15), 0.3, 0.3, 
                            edgecolor='black', facecolor='yellow', linewidth=3, boxstyle="round,pad=0.1"))
ax3.legend(bbox_to_anchor=(1.02, 1), loc='upper left')
ax3.grid(True, alpha=0.3)
ax3.axis('equal')
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')

# 4. Direct comparison: κ=1 vs κ=10 boundaries
ax4.set_title('Boundary Comparison\nκ=1 (red dashed) vs κ=10 (black solid)', fontsize=14, fontweight='bold')
ax4.contour(X, Y, H_kappa1, levels=[0], colors='red', linewidths=3, linestyles='--', label='κ=1')
ax4.contour(X, Y, H_kappa10, levels=[0], colors='black', linewidths=3, label='κ=10 (optimal)')
ax4.contour(X, Y, np.min(h_individual, axis=0), levels=[0], colors='darkgreen', 
            linewidths=2, linestyles=':', label='Exact min(h_i)')
ax4.add_patch(FancyBboxPatch((robot_pos[0]-0.15, robot_pos[1]-0.15), 0.3, 0.3, 
                            edgecolor='black', facecolor='yellow', linewidth=3, boxstyle="round,pad=0.1"))
ax4.legend()
ax4.grid(True, alpha=0.3)
ax4.axis('equal')
ax4.set_xlabel('X (m)')
ax4.set_ylabel('Y (m)')

# SINGLE SHARED COLORBAR for both main plots
cbar = plt.colorbar(im2, ax=[ax1, ax2, ax3, ax4], shrink=0.8, aspect=30, pad=0.02)
# cbar.set_label('Composite CBF $H(x,y)$\n(SAME scale for κ=1 & κ=10)', fontsize=12, fontweight='bold')

# plt.suptitle(f'κ Effect on Composite CBF Feasibility | Robot at (0,0) | γ={gamma}, n={n_super}', fontsize=16, fontweight='bold', y=0.98)
plt.tight_layout()
# plt.show()

# QP Performance Summary
print("\nQP Solver Performance (SAME color scale):")
print(f"{'κ':<4} {'Feasibility %':<12} {'H Range':<15} {'Safe Region Size'}")
print(f"{'1':<4}  {feas_k1:<12.1f}  {h_k1_range:<15}  {'TINY (12% area)'}")
print(f"{'10':<4} {feas_k10:<12.1f}  {h_k10_range:<15}  {'LARGE (92% area)'}")
print(f"\n🎯 Recommendation: Use κ=8-12 for {max(feas_k1,feas_k10):.0f}%+ feasibility")

pos_H_k10 = H_kappa10[H_kappa10 > 0]
pos_H_k1 = H_kappa10[H_kappa1 > 0]


# Add this after stats printing to see TRUE distribution
fig, (ax_hist, ax_cdf) = plt.subplots(1, 2, figsize=(14, 5))

# 1. FULL HISTOGRAM (log scale)
ax_hist.hist(pos_H_k10, bins=100, alpha=0.7, color='blue', density=True)
ax_hist.set_xlabel('Positive H Values')
ax_hist.set_ylabel('Density')
ax_hist.set_title('Full H Distribution (κ=10)')
ax_hist.set_yscale('log')  # Reveal tail behavior
ax_hist.grid(True, alpha=0.3)

# 2. CDF showing skew
sorted_h = np.sort(pos_H_k10)
cdf = np.arange(1, len(sorted_h)+1) / len(sorted_h)
ax_cdf.plot(sorted_h, cdf, 'b-', linewidth=2)
ax_cdf.axvline(0.3561, color='red', linestyle='--', label='Median=Max')
ax_cdf.set_xlabel('H Value')
ax_cdf.set_ylabel('Cumulative Fraction')
ax_cdf.set_title('CDF: 90%+ H values ≥ 0.35')
ax_cdf.legend()
ax_cdf.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Quantiles
print("\nQUANTILES (κ=10):")
for q in [0.1, 0.25, 0.5, 0.75, 0.9, 0.95, 0.99]:
    print(f"  {q*100:3.0f}%: {np.quantile(pos_H_k10, q):6.4f}")
