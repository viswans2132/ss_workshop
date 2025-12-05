import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle

# Composite CBF parameters
gamma = 0.4
kappa = 1.2
SUPER_ELLIPSE_ORDER = 4.0

# ROBOT POSITION at (0,0)
robot_pos = np.array([0.0, 0.0])

# 3 obstacles positioned AROUND the robot
obstacles = [
    {'center': (2.5, 0.5), 'a': 0.8, 'b': 1.2, 'color': 'red', 'name': 'Obstacle 1 (Front-Right)'},
    {'center': (-1.8, 1.2), 'a': 1.0, 'b': 0.9, 'color': 'blue', 'name': 'Obstacle 2 (Left)'},
    {'center': (0.8, -2.0), 'a': 1.2, 'b': 0.7, 'color': 'green', 'name': 'Obstacle 3 (Front-Bottom)'}
]

# Grid centered around robot
x = np.linspace(-3.5, 3.5, 400)
y = np.linspace(-3.0, 3.0, 400)
X, Y = np.meshgrid(x, y)

# Super-ellipse CBF function
def super_ellipse_cbf(X, Y, cx, cy, a, b, n):
    dx = np.abs(X - cx) / a
    dy = np.abs(Y - cy) / b
    return np.power(dx, n) + np.power(dy, n) - 1

# Compute individual h_i and composite H
h_individual = np.zeros((len(obstacles), *X.shape))
exp_terms = np.zeros((len(obstacles), *X.shape))

print(f"Robot at: {robot_pos}")
print(f"Computing super-ellipsoid CBF (n={SUPER_ELLIPSE_ORDER}) for {len(obstacles)} obstacles...")

for i, obs in enumerate(obstacles):
    cx, cy = obs['center']
    a, b = obs['a'], obs['b']
    
    # Individual CBF for obstacle i
    h_individual[i] = super_ellipse_cbf(X, Y, cx, cy, a, b, SUPER_ELLIPSE_ORDER)
    
    # Composite term
    term = -kappa * np.tanh(h_individual[i] / gamma)
    exp_terms[i] = np.exp(term)

# Composite CBF H
sum_exp = np.sum(exp_terms, axis=0)
H = -(gamma / kappa) * np.log(sum_exp + 1e-12)

print(f"H range: [{H.min():.3f}, {H.max():.3f}]")

# Custom colormap: red(danger) → blue(safe)
colors = ['red', 'orange', 'yellow', 'lightgreen', 'blue']
cmap = LinearSegmentedColormap.from_list('safety', colors, N=100)

# PLOTTING
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 14))

# 1. Individual CBF Contours + Robot
ax1.set_title('Individual Super-Ellipse CBFs (h_i = 0)', fontsize=14, fontweight='bold')
for i, obs in enumerate(obstacles):
    # h_i = 0 contour (safety boundary)
    ax1.contour(X, Y, h_individual[i], levels=[0], colors=obs['color'], linewidths=3, 
                label=f"{obs['name']}")
    # Fill unsafe region (h_i < 0, inside obstacle)
    ax1.contourf(X, Y, h_individual[i], levels=[-0.5, 0], alpha=0.3, colors=obs['color'])

# Robot position
robot_patch = Circle(robot_pos, 0.15, color='black', label='Robot (0,0)')
ax1.add_patch(robot_patch)
ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.axis('equal')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')

# 2. Distance from robot to each obstacle center
ax2.set_title('Distance to Obstacle Centers', fontsize=14, fontweight='bold')
for i, obs in enumerate(obstacles):
    cx, cy = obs['center']
    dist_field = (X - cx)**2 + (Y - cy)**2
    ax2.contourf(X, Y, dist_field, levels=15, alpha=0.6, cmap='viridis')
    ax2.contour(X, Y, dist_field, levels=[1.0], colors='white', linewidths=2, alpha=0.8)

robot_patch2 = Circle(robot_pos, 0.15, color='red', label='Robot')
ax2.add_patch(robot_patch2)
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.axis('equal')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')

# 3. Composite CBF with individuals overlaid
ax3.set_title('Composite CBF H(x) with Individual Contours', fontsize=14, fontweight='bold')
im3 = ax3.contourf(X, Y, H, levels=50, cmap=cmap, extend='both')
# Composite safety boundary
ax3.contour(X, Y, H, levels=[0], colors='black', linewidths=5, label='H=0 (Composite)')
# Individual contours faintly
for i, obs in enumerate(obstacles):
    ax3.contour(X, Y, h_individual[i], levels=[0], colors='gray', linewidths=1, alpha=0.6)

robot_patch3 = Circle(robot_pos, 0.15, color='magenta', label='Robot')
ax3.add_patch(robot_patch3)
ax3.legend()
ax3.grid(True, alpha=0.3)
ax3.axis('equal')
ax3.set_xlabel('X (m)')
ax3.set_ylabel('Y (m)')

# 4. Robot-centric view: Safe vs Unsafe regions
ax4.set_title('Safe (H≥0) vs Unsafe (H<0) Regions', fontsize=14, fontweight='bold')
im4 = ax4.contourf(X, Y, H, levels=[-0.5, 0, 1.0], colors=['red', 'lightgreen'], extend='both', alpha=0.8)
ax4.contour(X, Y, H, levels=[0], colors='black', linewidths=4)
ax4.contour(X, Y, H, levels=[0.5], colors='darkgreen', linewidths=2, linestyles='--', alpha=0.8)

robot_patch4 = Circle(robot_pos, 0.2, color='yellow', edgecolor='black', linewidth=3, label='Robot')
ax4.add_patch(robot_patch4)

# Safe/unsafe labels
ax4.text(-2.5, 1.5, 'UNSAFE\n(H < 0)', fontsize=12, ha='center', color='white', 
         bbox=dict(boxstyle='round', facecolor='red', alpha=0.7))
ax4.text(0.5, 1.5, 'SAFE\n(H ≥ 0)', fontsize=12, ha='center', color='black',
         bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))

ax4.legend()
ax4.grid(True, alpha=0.3)
ax4.axis('equal')
ax4.set_xlabel('X (m)')
ax4.set_ylabel('Y (m)')

# Colorbars
cbar3 = plt.colorbar(im3, ax=ax3, shrink=0.8)
cbar3.set_label('Composite CBF $H(x,y)$', fontsize=12)
cbar4 = plt.colorbar(im4, ax=ax4, shrink=0.8)
cbar4.set_label('H value', fontsize=12)

plt.suptitle(f'Composite Super-Ellipsoid CBF | Robot at ({robot_pos[0]}, {robot_pos[1]}) | γ={gamma}, κ={kappa}, n={SUPER_ELLIPSE_ORDER}', 
             fontsize=16, fontweight='bold')
plt.tight_layout()
plt.show()

# Print distances from robot to obstacles
print("\nRobot-Obstacle Distances:")
for i, obs in enumerate(obstacles):
    dist = np.linalg.norm(np.array(obs['center']) - robot_pos)
    print(f"  {obs['name']}: {dist:.2f}m")
