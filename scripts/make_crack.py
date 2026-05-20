"""
cracked_workpiece_generator.py
===============================
Generates a solid 3D steel workpiece STL file with a photorealistic hairline
fracture on its top surface, suitable for Gazebo simulation.

Pipeline:
  1. Define a flat 75×75×10 mm block on a regular 2D grid
  2. Simulate a wandering crack path using momentum-based random walk
  3. Carve the crack into the top surface using a Gaussian-style depth profile
  4. Visualise the fracture as a heatmap for inspection
  5. Build a fully closed, watertight solid mesh (6 faces: top, bottom, 4 walls)
  6. Save the result as a binary STL file for Gazebo / RViz
"""

import numpy as np
from stl import mesh
import os
import matplotlib.pyplot as plt


# =============================================================================
# SECTION 1 — WORKPIECE GEOMETRY PARAMETERS
# =============================================================================

length    = 75.0   # X extent of the plate (mm)
width     = 75.0   # Y extent of the plate (mm)
thickness = 10.0   # Z extent / plate depth (mm)
resolution = 0.25  # Grid spacing (mm); smaller = smoother surface, larger file

# Build a uniform 2D grid of X and Y sample positions
# np.arange generates values from 0 to length/width inclusive
x = np.arange(0, length + resolution, resolution)
y = np.arange(0, width  + resolution, resolution)

# meshgrid with indexing='ij': X[i, j] = x[i], Y[i, j] = y[j]
# rows = number of X samples, cols = number of Y samples
X, Y = np.meshgrid(x, y, indexing='ij')
rows, cols = X.shape


# =============================================================================
# SECTION 2 — CRACK PATH GENERATION (MOMENTUM-BASED RANDOM WALK)
# =============================================================================

# Z_top stores the elevation of every grid point on the top surface.
# Initially flat at Z = 0; crack points will be pushed negative (into the plate).
Z_top = np.zeros_like(X)

np.random.seed(101)  # Fix seed for reproducibility — same crack every run

# path_y[i] = the Y-coordinate of the crack centre at each X slice (column i)
path_y  = np.zeros(rows)
current_y = 37.5   # Start the crack at the horizontal centre of the plate (mm)
momentum  = 0.0    # Running momentum term — makes the path curve gradually

for i in range(rows):
    path_y[i] = current_y

    # Momentum decays each step (factor 0.7) and is nudged by Gaussian noise.
    # Low noise (std=0.3) keeps the crack relatively straight but organically
    # curved, mimicking a real weld-heat-affected-zone fracture.
    momentum   = 0.7 * momentum + np.random.normal(0, 0.3)
    current_y += momentum


# =============================================================================
# SECTION 3 — CARVE THE FRACTURE INTO THE TOP SURFACE
# =============================================================================

print("Generating solid workpiece with hairline fracture...")

for i in range(rows):
    center_y = path_y[i]  # Crack centre Y for this X slice

    # local_width varies slightly per slice to mimic irregular crack edges.
    # Base width = 0.4 mm; abs(Normal(0, 0.3)) adds random widening.
    local_width = 0.4 + np.abs(np.random.normal(0, 0.3))

    max_depth = 2.0  # Maximum crack depth (mm) at the centre line

    # --- Vectorised distance computation for the entire row at once ---
    # dist[j] = perpendicular distance from grid point Y[i,j] to crack centre
    dist = np.abs(Y[i, :] - center_y)

    # Boolean mask: True for grid points that fall inside the crack width
    mask = dist < local_width

    # Only process points that lie within the crack boundary
    if np.any(mask):
        # depth_ratio goes from 1.0 at the crack centre to 0.0 at its edges.
        # This creates a smooth V-profile cross-section (deepest in the middle).
        depth_ratio = 1.0 - (dist[mask] / local_width)

        # Add small Gaussian noise to simulate rough, irregular crack walls
        noise = np.random.normal(0, 0.1, size=np.sum(mask))

        # Final depth = parabolic profile + noise; always negative (below surface)
        local_depth = (max_depth * depth_ratio) + noise
        Z_top[i, mask] = -np.abs(local_depth)


# =============================================================================
# SECTION 4 — FRACTURE HEATMAP VISUALISATION
# =============================================================================

# Transpose Z_top so X runs horizontally and Y runs vertically in the image.
# origin='lower' places Y=0 at the bottom, matching the physical orientation.
plt.imshow(Z_top.T, cmap='terrain', origin='lower')
plt.colorbar(label='Depth (mm)')
plt.title("Workpiece Fracture Heatmap")
plt.xlabel("X grid index")
plt.ylabel("Y grid index")
plt.tight_layout()
plt.show()


# =============================================================================
# SECTION 5 — BUILD CLOSED SOLID MESH
# =============================================================================

# The flat bottom surface sits at a constant Z = -thickness (e.g. Z = -10 mm)
Z_bottom = -thickness * np.ones_like(X)

# Accumulate all triangle vertices and face index triplets
vertices = []
faces    = []


def add_quad(p1, p2, p3, p4, flip=False):
    """
    Adds two triangles that together form one rectangular quad face.

    The two triangles share the diagonal p2–p3:
      Triangle A: p1, p2, p3
      Triangle B: p4, p3, p2  (or reversed when flip=True)

    Parameters
    ----------
    p1, p2, p3, p4 : array-like [x, y, z]
        Four corner points of the quad in order:
        p1 ── p2
        │      │
        p3 ── p4
    flip : bool
        If False → normals point in the "default" winding direction (outward for
                    surfaces facing +Z or the +X/+Y sides).
        If True  → winding order is reversed so normals flip 180°, required for
                    surfaces whose outward normal faces −Z or the opposite walls.
    """
    v_idx = len(vertices)
    vertices.extend([p1, p2, p3, p4])

    if flip:
        # Reversed winding: normal points the opposite way
        faces.extend([
            [v_idx,     v_idx + 2, v_idx + 1],
            [v_idx + 3, v_idx + 1, v_idx + 2]
        ])
    else:
        # Default winding: normal points outward for this face orientation
        faces.extend([
            [v_idx,     v_idx + 1, v_idx + 2],
            [v_idx + 2, v_idx + 1, v_idx + 3]
        ])


# --- Face 1: Top Surface (normals point UP / +Z) ---
# Iterates over every 2×2 cell of the grid and tessellates it into 2 triangles.
# flip=False because the default winding gives an upward-facing normal.
for i in range(rows - 1):
    for j in range(cols - 1):
        add_quad(
            [X[i,   j],   Y[i,   j],   Z_top[i,   j]  ],
            [X[i+1, j],   Y[i+1, j],   Z_top[i+1, j]  ],
            [X[i,   j+1], Y[i,   j+1], Z_top[i,   j+1]],
            [X[i+1, j+1], Y[i+1, j+1], Z_top[i+1, j+1]],
            flip=False
        )

# --- Face 2: Bottom Surface (normals point DOWN / −Z) ---
# Same grid tessellation as the top, but flip=True reverses the winding so
# normals face downward (outward from the solid's underside).
for i in range(rows - 1):
    for j in range(cols - 1):
        add_quad(
            [X[i,   j],   Y[i,   j],   Z_bottom[i,   j]  ],
            [X[i+1, j],   Y[i+1, j],   Z_bottom[i+1, j]  ],
            [X[i,   j+1], Y[i,   j+1], Z_bottom[i,   j+1]],
            [X[i+1, j+1], Y[i+1, j+1], Z_bottom[i+1, j+1]],
            flip=True
        )

# --- Face 3: Front Wall — Y = 0 edge (normals point toward −Y) ---
# Connects the bottom edge at j=0 to the (fractured) top edge at j=0.
# flip=False gives the correct outward normal for this orientation.
for i in range(rows - 1):
    add_quad(
        [X[i,   0], Y[i,   0], Z_bottom[i,   0]],
        [X[i+1, 0], Y[i+1, 0], Z_bottom[i+1, 0]],
        [X[i,   0], Y[i,   0], Z_top[i,   0]   ],
        [X[i+1, 0], Y[i+1, 0], Z_top[i+1, 0]   ],
        flip=False
    )

# --- Face 4: Back Wall — Y = width edge (normals point toward +Y) ---
# j = -1 selects the last column (Y = width boundary).
# flip=True reverses winding so the normal faces outward (+Y direction).
for i in range(rows - 1):
    add_quad(
        [X[i,   -1], Y[i,   -1], Z_bottom[i,   -1]],
        [X[i+1, -1], Y[i+1, -1], Z_bottom[i+1, -1]],
        [X[i,   -1], Y[i,   -1], Z_top[i,   -1]   ],
        [X[i+1, -1], Y[i+1, -1], Z_top[i+1, -1]   ],
        flip=True
    )

# --- Face 5: Left Wall — X = 0 edge (normals point toward −X) ---
# i = 0 selects the first row (X = 0 boundary).
# flip=True gives the correct outward normal for the −X face.
for j in range(cols - 1):
    add_quad(
        [X[0, j  ], Y[0, j  ], Z_bottom[0, j  ]],
        [X[0, j+1], Y[0, j+1], Z_bottom[0, j+1]],
        [X[0, j  ], Y[0, j  ], Z_top[0, j  ]   ],
        [X[0, j+1], Y[0, j+1], Z_top[0, j+1]   ],
        flip=True
    )

# --- Face 6: Right Wall — X = length edge (normals point toward +X) ---
# i = -1 selects the last row (X = length boundary).
# flip=False gives the correct outward normal for the +X face.
for j in range(cols - 1):
    add_quad(
        [X[-1, j  ], Y[-1, j  ], Z_bottom[-1, j  ]],
        [X[-1, j+1], Y[-1, j+1], Z_bottom[-1, j+1]],
        [X[-1, j  ], Y[-1, j  ], Z_top[-1, j  ]   ],
        [X[-1, j+1], Y[-1, j+1], Z_top[-1, j+1]   ],
        flip=False
    )


# =============================================================================
# SECTION 6 — ASSEMBLE AND EXPORT STL
# =============================================================================

# Convert Python lists to NumPy arrays for fast indexing
vertices = np.array(vertices)
faces    = np.array(faces)

# Allocate an empty numpy-stl Mesh with one entry per triangle face
workpiece_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))

# Fill each triangle by looking up its three vertices from the faces index array
for i, f in enumerate(faces):
    for j in range(3):
        workpiece_mesh.vectors[i][j] = vertices[f[j], :]

# Save to the Gazebo meshes directory (created if it does not exist)
save_path = '../meshes/gazebo_cracked_workpiece.stl'
os.makedirs(os.path.dirname(save_path), exist_ok=True)
workpiece_mesh.save(save_path)

print(f"SUCCESS: Solid watertight STL saved → {save_path}")
print(f"  Grid  : {rows} × {cols} points  ({resolution} mm resolution)")
print(f"  Faces : {faces.shape[0]} triangles")