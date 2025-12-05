import numpy as np
import matplotlib.pyplot as plt

# Parameters
gamma = 1.0
kappa = 10.0
n_super = 8.0
robot_pos = np.array([0.0, 0.0])
obstacles = [
    {'center': (2.5, 0.5), 'a': 0.8, 'b': 1.2},
    {'center': (-1.8, 1.2), 'a': 1.0, 'b': 0.9},
    {'center': (0.8, -2.0), 'a': 1.2, 'b': 0.7}
]

# Grid
x = np.linspace(-3.5, 3.5, 400)
y = np.linspace(-3.0, 3.0, 400)
X, Y = np.meshgrid(x, y)

def super_ellipse_cbf(X, Y, cx, cy, a, b, n):
    dx = np.abs(X - cx) / a
    dy = np.abs(Y - cy) / b
    return np.power(dx, n) + np.power(dy, n) - 1

# Compute individual h_i
print("Computing individual super-ellipse CBFs...")
h_individual = np.zeros((len(obstacles), *X.shape))
for i, obs in enumerate(obstacles):
    cx, cy = obs['center']
    a, b = obs['a'], obs['b']
    h_individual[i] = super_ellipse_cbf(X, Y, cx, cy, a, b, n_super)

# FIXED COMPOSITE FUNCTIONS (all return 2D arrays)
def composite_with_tanh(h, kappa, gamma):
    exp_terms = np.exp(-kappa * np.tanh(h / gamma))
    sum_exp = np.sum(exp_terms, axis=0)
    return -(gamma / kappa) * np.log(sum_exp + 1e-12)

def composite_raw_unstable(h, kappa, gamma):
    try:
        exp_terms = np.exp(-kappa * h / gamma)
        sum_exp = np.sum(exp_terms, axis=0)
        return -(gamma / kappa) * np.log(sum_exp + 1e-12)
    except:
        return np.full(h.shape[1:], np.nan)

def composite_raw_stable(h, kappa, gamma):
    scaled = -kappa * h / gamma
    max_scaled = np.max(scaled, axis=0, keepdims=True)
    lse = max_scaled + np.log(np.sum(np.exp(scaled - max_scaled), axis=0) + 1e-12)
    return -(gamma / kappa) * lse.squeeze()

# Compute all versions
H_tanh = composite_with_tanh(h_individual, kappa, gamma)
H_raw_unstable = composite_raw_unstable(h_individual, kappa, gamma)
H_raw_stable = np.clip(composite_raw_stable(h_individual, kappa, gamma), -2.0, 2.0)

# SHARED COLOR SCALE: Global min/max across ALL plots
global_min = min(H_tanh.min(), H_raw_stable.min(), np.nanmin(H_raw_unstable))
global_max = max(H_tanh.max(), H_raw_stable.max(), np.nanmax(H_raw_unstable))
SHARED_LEVELS = np.linspace(global_min, global_max, 30)
print(f"SHARED COLOR SCALE: [{global_min:.3f}, {global_max:.3f}]")

# Stats
def safe_feasibility(H):
    H_valid = H[~np.isnan(H)]
    if len(H_valid) == 0:
        return 0.0
    return 100 * np.sum(H_valid >= 0) / len(H_valid)

print("=== COMPARISON WITH SHARED SCALE ===")
print(f"With tanh:     {safe_feasibility(H_tanh):5.1f}% | Range: [{H_tanh.min():6.3f}, {H_tanh.max():6.3f}]")
print(f"Raw unstable:  {safe_feasibility(H_raw_unstable):5.1f}% | NaNs: {np.sum(np.isnan(H_raw_unstable)):,}")
print(f"Raw stable:    {safe_feasibility(H_raw_stable):5.1f}% | Range: [{H_raw_stable.min():6.3f}, {H_raw_stable.max():6.3f}]")

# PLOT: SAME SCALE FOR ALL
fig, axes = plt.subplots(2, 2, figsize=(16, 12))

titles = ['✅ WITH tanh', '❌ RAW Unstable', '⚠️ Raw Stable (clipped)', 'Raw Stable - tanh']
H_plots = [H_tanh, H_raw_unstable, H_raw_stable, H_raw_stable - H_tanh]

for i, (ax, H_plot, title) in enumerate(zip(axes.flat, H_plots, titles)):
    # SAME levels for ALL plots
    im = ax.contourf(X, Y, H_plot, levels=SHARED_LEVELS, cmap='RdYlGn', extend='both')
    ax.contour(X, Y, H_plot, [0], colors='black', linewidths=4, label='H=0')
    ax.plot(robot_pos[0], robot_pos[1], 'yo', markersize=15, markeredgecolor='k', linewidth=3)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)

# SINGLE SHARED COLORBAR
cbar = plt.colorbar(im, ax=axes.ravel().tolist(), shrink=0.8, aspect=30, pad=0.02)
cbar.set_label('Composite CBF $H(x,y)$\n(SAME SCALE FOR ALL PLOTS)', fontsize=12, fontweight='bold')
cbar.ax.axhline(0, color='k', linestyle='-', linewidth=2)  # H=0 line on colorbar

# plt.suptitle(f'With vs Without tanh | SAME COLOR SCALE [{global_min:.2f}, {global_max:.2f}] | κ={kappa}, γ={gamma}', 
#              fontsize=16, fontweight='bold')
plt.tight_layout()
plt.show()

# H AT ROBOT POSITION
robot_idx = (200, 200)
print(f"\nH at Robot (0,0) [row={robot_idx[0]}, col={robot_idx[1]}]:")
print(f"  With tanh:     {H_tanh[robot_idx]:+7.3f}")
print(f"  Raw unstable:  {H_raw_unstable[robot_idx]:+7.3f}")
print(f"  Raw stable:    {H_raw_stable[robot_idx]:+7.3f}")
