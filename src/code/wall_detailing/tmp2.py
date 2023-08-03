import os

import OCC
from OCC.Core import BRepPrimAPI, BRepAlgo
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepGProp import brepgprop_LinearProperties
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Pnt, gp_Ax2, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.BRep import BRep_Tool
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop_LinearProperties
from OCC.Core.gp import gp_Pnt
from OCC.Core.StlAPI import StlAPI_Writer

f = BRepAlgoAPI_Fuse(
    BRepPrimAPI_MakeCylinder(6.0, 100).Shape(),
    BRepPrimAPI_MakeSphere(6.0).Shape()
).Shape()
# Create a mesh representation of the shape
mesh = BRepMesh_IncrementalMesh(f, 0.1)

# Generate the mesh
mesh.Perform()
assert mesh.IsDone()
directory = os.path.split(__name__)[0]
stl_output_dir = os.path.abspath(directory)
assert os.path.isdir(stl_output_dir)

stl_file = os.path.join(stl_output_dir, "tmp2.stl")

stl_export = StlAPI_Writer()
print(stl_export.Write(mesh.Shape(), stl_file))
