import os
import pickle
from typing import List, Dict
import numpy as np
import quaternion
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Ax3, gp_XYZ

rotation = quaternion.from_euler_angles(45,0,0)
corner = gp_Pnt(1, 2, 3)

# Extract the scalar (real) and vector components from the NumPy quaternion
scalar_part = rotation.w
vector_part = np.array([rotation.x, rotation.y, rotation.z])
# Create the gp_Quaternion object from the scalar and vector components
rotation = gp_Quaternion(scalar_part, *vector_part)

# Convert the shape to a gp_Trsf transformation
transformation = gp_Trsf()
transformation.SetRotation(rotation)

shape = BRepPrimAPI_MakeBox(corner, 5, 5, 5).Shape()
shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
print(shape.Location().Transformation().GetRotation().X(),
      shape.Location().Transformation().GetRotation().Y(),
      shape.Location().Transformation().GetRotation().Z(),
      shape.Location().Transformation().GetRotation().W())
shape.Location().Transformation().SetRotation(rotation)
print(shape.Location().Transformation().GetRotation().X(),
      shape.Location().Transformation().GetRotation().Y(),
      shape.Location().Transformation().GetRotation().Z(),
      shape.Location().Transformation().GetRotation().W())
print(shape.Location().Transformation().TranslationPart().X(),
      shape.Location().Transformation().TranslationPart().Y(),
      shape.Location().Transformation().TranslationPart().Z())

mesh = BRepMesh_IncrementalMesh(shape, 0.1)

# Generate the mesh
mesh.Perform()
assert mesh.IsDone()
directory = os.path.split(__name__)[0]
stl_output_dir = os.path.abspath(directory)
assert os.path.isdir(stl_output_dir)

stl_file = os.path.abspath("tmp3.stl")

stl_export = StlAPI_Writer()
print(stl_export.Write(mesh.Shape(), stl_file))

a = pickle.dumps(shape)