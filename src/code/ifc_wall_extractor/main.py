import ifcopenshell
from ifcopenshell import geom
from stl import mesh, Mode
import numpy as np
from OCC import VERSION

print("OCC version", VERSION)

# Specify to return pythonOCC shapes from ifcopenshell.geom.create_shape()
# settings.set(settings.USE_PYTHON_OPENCASCADE, True)

settings = geom.settings()
settings.set(settings.USE_WORLD_COORDS, True)

ifc_file = ifcopenshell.open("../../models/lego_wall_with_door_tmp_random.ifc")
products = ifc_file.by_type("IfcProduct")
meshes = []

for product in products:
    if product.Representation and (product.is_a("IfcWall")):
        shape = geom.create_shape(settings, product)
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
