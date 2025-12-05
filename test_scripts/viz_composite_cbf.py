import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# Composite CBF parameters
gamma = 0.4
kappa = 1.0

# Super-ellipse order (n=8 gives box-like, n=4 is ellipse, n=2 is circle)
SUPER_ELLIPSE_ORDER = 4.0

# 3 example super-ellipses (oriented obstacle safety regions)
obstacles = [
    {'center': (0.0, 0.0), 'a': 1.2, 'b': 0.8, 'color': 'blue'},   
    {'center': (2.0, 1.0), 'a': 1.0, 'b': 1.5, 'color': 'red'},    
    {'center': (-1.5, -1.0), 'a': 0.8, 'b': 1.2, 'color': 'green'}
]

# Grid
x = np.linspace(-4, 4, 300)
y = np.linspace(-3, 3, 300)
X, Y = np.meshgrid(x, y)

# Super-ellipse CBF: h_i = |X/a|^n + |Y/b|^n - 1
def super_ellipse_cbf(X, Y, cx, cy, a, b, n):
    dx = np.abs(X - cx) / a
    dy = np.abs(Y - cy) / b
    return np.power(dx, n) + np.power(dy, n) - 1

# Compute composite H
H = np.zeros_like(X)
exp_terms = np.zeros((len(obstacles), *X.shape))

print(f"Computing super-ellipsoid CBF (order n={SUPER_ELLIPSE_ORDER})...")
for i, obs in enumerate(obstacles):
    cx, cy = obs['center']
    a, b = obs['a'], obs['b']
    
    # Super-ellipse CBF
    h_i = super_ellipse_cbf(X, Y, cx, cy, a, b, SUPER_ELLIPSE_ORDER)
    
    # Composite term: exp[-κ * tanh(h_i/γ)]
    term = -kappa * np.tanh(h_i / gamma)
    exp_terms[i] = np.exp(term)

sum_exp = np.sum(exp_terms, axis=0)
H = -(gamma / kappa) * np.log(sum_exp + 1e-12)

print(f"H range: [{H.min():.3f}, {H.max():.3f}]")

# Create custom colormap (red=unsafe, blue=safe)
colors = ['red', 'orange', 'yellow', 'green', 'blue']
cmap = LinearSegmentedColormap.from_list('safety', colors, N=100)

# Plotting: 2x4 grid to fit all subplots
fig, axes = plt.subplots(1, 2, figsize=(20, 10))

# 1. Individual super-ellipses
ax = axes[0]
ax.set_title(f'Individual Super-Ellipses (n={SUPER_ELLIPSE_ORDER})', fontsize=14, fontweight='bold')
for i, obs in enumerate(obstacles):
    cx, cy = obs['center']
    a, b = obs['a'], obs['b']
    h_i = super_ellipse_cbf(X, Y, cx, cy, a, b, SUPER_ELLIPSE_ORDER)
    ax.contour(X, Y, h_i, levels=[0], colors=obs['color'], linewidths=3)
    ax.contourf(X, Y, h_i, levels=[-0.5, 0], alpha=0.3, colors=obs['color'])
ax.grid(True, alpha=0.3)
ax.axis('equal')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# 2. Composite CBF
ax = axes[1]
im = ax.contourf(X, Y, H, levels=30, cmap=cmap, extend='both')
ax.contour(X, Y, H, levels=[0], colors='black', linewidths=4)  # Fixed: removed label
ax.set_title('Composite Super-Ellipse CBF', fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.axis('equal')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# Overlay individuals faintly on composite
for obs in obstacles:
    cx, cy = obs['center']
    a, b = obs['a'], obs['b']
    h_i = super_ellipse_cbf(X, Y, cx, cy, a, b, SUPER_ELLIPSE_ORDER)
    axes[1].contour(X, Y, h_i, levels=[0], colors='gray', linewidths=1, alpha=0.5)


# Colorbar for composite plot
cbar = plt.colorbar(im, ax=axes[0], shrink=0.8)
cbar.set_label('Composite CBF Value $H(x,y)$', rotation=270, labelpad=20, fontsize=12)

plt.suptitle(f'Composite Super-Ellipsoid CBF (γ={gamma}, κ={kappa}, n={SUPER_ELLIPSE_ORDER})', 
             fontsize=16, fontweight='bold', y=0.98)
plt.tight_layout()
plt.show()

# Print key stats
print("\nComposite CBF Properties:")
print(f"γ = {gamma}, κ = {kappa}")
print(f"Super-ellipse order n = {SUPER_ELLIPSE_ORDER}")
print(f"Safety region: H(x,y) ≥ 0 (inside black contour)")
print(f"Unsafe region: H(x,y) < 0 (red areas)")
print("\nSubplot layout:")
print("- Top-left: Individual super-ellipses")
print("- Top-right: Composite CBF H(x)")
print("- Bottom: Single obstacle at different orders n=2,4,8,16")
