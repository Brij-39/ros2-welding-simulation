import numpy as np
from stl import mesh
import os

# --- SOLID WORKPIECE PARAMETERS ---
length = 75.0
width = 75.0
thickness = 10.0
resolution = 0.25

x = np.arange(0, length + resolution, resolution)
y = np.arange(0, width + resolution, resolution)
X, Y = np.meshgrid(x, y, indexing='ij')
rows, cols = X.shape

Z_top = np.zeros_like(X)

# --- PHOTO REALISTIC FRACTURE PATH ---
np.random.seed(101)
path_y = np.zeros(rows)
current_y = 37.5
momentum = 0.0

for i in range(rows):
    path_y[i] = current_y
    momentum = 0.7 * momentum + np.random.normal(0, 0.3)
    current_y += momentum

print("Generating solid workpiece with hairline fracture...")
for i in range(rows):
    center_y = path_y[i]
    local_width = 0.4 + np.abs(np.random.normal(0, 0.3)) 
    max_depth = 2.0
    
    # 1. Ek hi baar mein poore row ka distance check karna
    dist = np.abs(Y[i, :] - center_y)
    mask = dist < local_width  # Jo points crack ke andar hain, unko mark karna
    
    # 2. Sirf un marked points par crack banana
    if np.any(mask):
        depth_ratio = 1.0 - (dist[mask] / local_width)
        noise = np.random.normal(0, 0.1, size=np.sum(mask))
        local_depth = (max_depth * depth_ratio) + noise
        
        Z_top[i, mask] = -np.abs(local_depth)
import matplotlib.pyplot as plt

plt.imshow(Z_top.T, cmap='terrain', origin='lower')
plt.colorbar(label='Depth')
plt.title("Workpiece Fracture Heatmap")
plt.show()
Z_bottom = -thickness * np.ones_like(X)

# --- CREATE 3D SOLID BLOCK (WITH CORRECT NORMALS) ---
vertices = []
faces = []

# Flip parameter added to point walls outward!
def add_quad(p1, p2, p3, p4, flip=False):
    v_idx = len(vertices)
    vertices.extend([p1, p2, p3, p4])
    if flip:
        faces.extend([[v_idx, v_idx+2, v_idx+1], [v_idx+3, v_idx+1, v_idx+2]])
    else:
        faces.extend([[v_idx, v_idx+1, v_idx+2], [v_idx+2, v_idx+1, v_idx+3]])

# 1. Top Surface (Points UP -> flip=False)
for i in range(rows - 1):
    for j in range(cols - 1):
        add_quad([X[i, j], Y[i, j], Z_top[i, j]], [X[i+1, j], Y[i+1, j], Z_top[i+1, j]],
                 [X[i, j+1], Y[i, j+1], Z_top[i, j+1]], [X[i+1, j+1], Y[i+1, j+1], Z_top[i+1, j+1]], flip=False)

# 2. Bottom Surface (Points DOWN -> flip=True)
for i in range(rows - 1):
    for j in range(cols - 1):
        add_quad([X[i, j], Y[i, j], Z_bottom[i, j]], [X[i+1, j], Y[i+1, j], Z_bottom[i+1, j]],
                 [X[i, j+1], Y[i, j+1], Z_bottom[i, j+1]], [X[i+1, j+1], Y[i+1, j+1], Z_bottom[i+1, j+1]], flip=True)

# 3. Front Wall (Points outward -> flip=False)
for i in range(rows - 1):
    add_quad([X[i, 0], Y[i, 0], Z_bottom[i, 0]], [X[i+1, 0], Y[i+1, 0], Z_bottom[i+1, 0]],
             [X[i, 0], Y[i, 0], Z_top[i, 0]], [X[i+1, 0], Y[i+1, 0], Z_top[i+1, 0]], flip=False)

# 4. Back Wall (Points outward -> flip=True)
for i in range(rows - 1):
    add_quad([X[i, -1], Y[i, -1], Z_bottom[i, -1]], [X[i+1, -1], Y[i+1, -1], Z_bottom[i+1, -1]],
             [X[i, -1], Y[i, -1], Z_top[i, -1]], [X[i+1, -1], Y[i+1, -1], Z_top[i+1, -1]], flip=True)

# 5. Left Wall (Points outward -> flip=True)
for j in range(cols - 1):
    add_quad([X[0, j], Y[0, j], Z_bottom[0, j]], [X[0, j+1], Y[0, j+1], Z_bottom[0, j+1]],
             [X[0, j], Y[0, j], Z_top[0, j]], [X[0, j+1], Y[0, j+1], Z_top[0, j+1]], flip=True)

# 6. Right Wall (Points outward -> flip=False)
for j in range(cols - 1):
    add_quad([X[-1, j], Y[-1, j], Z_bottom[-1, j]], [X[-1, j+1], Y[-1, j+1], Z_bottom[-1, j+1]],
             [X[-1, j], Y[-1, j], Z_top[-1, j]], [X[-1, j+1], Y[-1, j+1], Z_top[-1, j+1]], flip=False)

vertices = np.array(vertices)
faces = np.array(faces)
workpiece_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(faces):
    for j in range(3):
        workpiece_mesh.vectors[i][j] = vertices[f[j], :]

save_path = '../meshes/gazebo_cracked_workpiece.stl'
os.makedirs(os.path.dirname(save_path), exist_ok=True)
workpiece_mesh.save(save_path)
print("SUCCESS: Solid Box is Ready!")