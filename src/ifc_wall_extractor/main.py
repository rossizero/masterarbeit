import ifcopenshell
from ifcopenshell import geom
from stl import mesh
from OCC import VERSION
import numpy as np

print("OCC version", VERSION)

# Specify to return pythonOCC shapes from ifcopenshell.geom.create_shape()
settings = geom.settings()
#settings.set(settings.USE_PYTHON_OPENCASCADE, True)

# Open the IFC file using IfcOpenShell
ifc_file = ifcopenshell.open("../models/sample_house.ifc")

# Display the geometrical contents of the file using Python OpenCascade
products = ifc_file.by_type("IfcWall")
print(products)
vertices = []
faces = []
edges = []

for product in products[-2:-1]:
    shape = ifcopenshell.geom.create_shape(settings, product)
    print(shape.geometry.verts, shape.geometry.faces)
    vertices.extend(shape.geometry.verts)
    edges.extend(shape.geometry.edges)
    faces.extend(shape.geometry.faces)

vertices = np.array(vertices).reshape((-1, 3))
faces = np.array(faces).reshape((-1, 3))

# Create the mesh
cube = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(faces):
    for j in range(3):
        cube.vectors[i][j] = vertices[f[j],:]

# Write the mesh to file "cube.stl"
cube.save('cube.stl')
