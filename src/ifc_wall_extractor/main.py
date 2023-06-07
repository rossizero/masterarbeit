import ifcopenshell
from ifcopenshell import geom
from stl import mesh, Mode
from OCC import VERSION
import numpy as np

print("OCC version", VERSION)

# Specify to return pythonOCC shapes from ifcopenshell.geom.create_shape()
# settings.set(settings.USE_PYTHON_OPENCASCADE, True)

settings = geom.settings()
settings.set(settings.USE_WORLD_COORDS, True)

# Open the IFC file using IfcOpenShell
ifc_file = ifcopenshell.open("../models/sample_house.ifc")

# Display the geometrical contents of the file using Python OpenCascade
products = ifc_file.by_type("IfcProduct")
print(products)
meshes = []

for product in products:
    if product.is_a("IfcOpeningElement"):
        print("moin")
    if product.Representation and (product.is_a("IfcSite") or product.is_a("IfcStair") or product.is_a("IfcWall")):
        print(product)
        shape = ifcopenshell.geom.create_shape(settings, product)
        vertices = np.array(shape.geometry.verts).reshape((-1, 3))
        edges = np.array(shape.geometry.edges)
        faces = np.array(shape.geometry.faces).reshape((-1, 3))

        m = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
        for i, f in enumerate(faces):
            for j in range(3):
                m.vectors[i][j] = vertices[f[j], :]
        meshes.append(m)

# Create the combined mesh
combined = mesh.Mesh(np.concatenate([m.data for m in meshes]))
combined.save('cube.stl', mode=Mode.ASCII)
