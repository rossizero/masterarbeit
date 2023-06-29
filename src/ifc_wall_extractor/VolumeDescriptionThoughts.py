# conda install scipy
# conda install -c conda-forge matplotlib

import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
from stl import mesh, Mode



# Beispiel-Liste von Ebenen in Hessescher Normalform
planes = [
    [1, 0, 0, 0.5],
    [1, 0, 0, -0.5],
    [0, 1, 0, 0.5],
    [0, 1, 0, -0.5],
    [0, 0, 1, 0.5],
    [0, 0, 1, -0.5],
]

# Beispiel-Liste von Ebenen in Hessescher Normalform
planes = [
    # cube
    [1, 0, 0, -1.0],
    [1, 0, 0, 1.0],
    [0, 1, 0, 0.5],
    [0, 1, 0, -0.5],
    [0, 0, 1, 0.5],
    [0, 0, 1, -0.5],
    # L shape
    [1, 0, 0, 0],
    [0, 0, 1, 2],
]

# Extrahieren der Normalenvektoren und D-Koeffizienten der Ebenen
normals = np.array([plane[:3] for plane in planes])
d_coeffs = np.array([plane[3] for plane in planes])

# Schnittpunkte der Ebenen finden
intersections = []
num_planes = len(planes)
for i in range(num_planes):
    for j in range(i+1, num_planes):
        for k in range(j+1, num_planes):
            A = np.vstack((normals[i], normals[j], normals[k]))
            b = np.array([-d_coeffs[i], -d_coeffs[j], -d_coeffs[k]])
            #print(A, b, np.linalg.det(A))
            if np.linalg.det(A) > 0:
                intersection = np.linalg.solve(A, b)
                intersections.append(intersection)

print(intersections)

# Konvexe Hülle der Schnittpunkte berechnen
hull = ConvexHull(intersections)

# Volumen berechnen
volume = hull.volume

# 3D-Plot erstellen
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

intersections = np.array(intersections)
x, y, z = intersections.T
ax.scatter(x, y, z)


# Schnittpunkte der konvexen Hülle hinzufügen

faces = np.zeros(shape=(len(hull.simplices), 3))
for i, s in enumerate(hull.simplices):
    face = [[s, 0], [s, 1], [s, 2]]
    faces[i] = s
    s = np.append(s, s[0])  # Here we cycle back to the first coordinate
    ax.plot(intersections[s, 0], intersections[s, 1], intersections[s, 2], "r-")
    #print(face)

# Achsenbeschriftungen hinzufügen
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.title('Konvexe Hülle')
plt.show()

print("Das Volumen des Körpers beträgt:", volume)
print(len(intersections))
#print(intersections, faces)
m = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
print(intersections)
print(faces)
for i, f in enumerate(faces):
    for j in range(3):
        # TODO L shape is not working -> displays a cube
        m.vectors[i][j] = intersections[int(f[j]), :]

m.save('tile.stl', mode=Mode.ASCII)