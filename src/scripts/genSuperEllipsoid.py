import trimesh
import math

# Superellipsoid parameters
a, b, c = 1.0, 1.0, 1.0
n = 4.0
Nu, Nv = 80, 120

def sgn(x):
    return -1.0 if x < 0 else 1.0

vertices = []
faces = []

# Generate vertices
for i in range(Nu):
    u = -math.pi/2 + math.pi * i / (Nu-1)
    cu, su = math.cos(u), math.sin(u)
    cu_n = abs(cu)**(2/n)
    su_n = abs(su)**(2/n)
    for j in range(Nv):
        v = -math.pi + 2*math.pi * j / (Nv-1)
        cv, sv = math.cos(v), math.sin(v)
        cv_n = abs(cv)**(2/n)
        sv_n = abs(sv)**(2/n)
        x = a * sgn(cu) * cu_n * sgn(cv) * cv_n
        y = b * sgn(cu) * cu_n * sgn(sv) * sv_n
        z = c * sgn(su) * su_n
        vertices.append([x, y, z])

# Generate faces
for i in range(Nu-1):
    for j in range(Nv-1):
        v0 = i*Nv + j
        v1 = (i+1)*Nv + j
        v2 = (i+1)*Nv + (j+1)
        v3 = i*Nv + (j+1)
        faces.append([v0, v1, v2])
        faces.append([v0, v2, v3])

mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
mesh.export("superellipsoid4.stl")
