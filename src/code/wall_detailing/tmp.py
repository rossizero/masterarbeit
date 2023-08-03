import os

import numpy as np
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepGProp import brepgprop_LinearProperties
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import topods
from OCC.Core.gp import gp_Pnt, gp_Ax2, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.BRep import BRep_Tool
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop_LinearProperties
from OCC.Core.gp import gp_Pnt
from OCC.Core.StlAPI import StlAPI_Writer

# Create the first box
corner1_box1 = gp_Pnt(1, 2, 3)
box1 = BRepPrimAPI_MakeBox(corner1_box1, 5, 5, 10).Shape()

# Create the second box
corner1_box2 = gp_Pnt(10, 10, 10)
corner2_box2 = gp_Pnt(20, 20, 40)
box2 = BRepPrimAPI_MakeBox(corner1_box2, corner2_box2).Shape()

# Perform the boolean operation to fuse the two boxes into an L-shaped form
l_shaped_form = BRepAlgoAPI_Fuse(box1, box2).Shape()


# Assuming you have 'box' as the BRep shape created in the previous step.

# Create a GProps object to store the linear properties
props = GProp_GProps()

# Initialize the total length to zero
total_length = 0.0

# Get the explorer to access the edges of the shape
explorer = TopExp_Explorer(box1, TopAbs_EDGE)

while explorer.More():
    # Get the current edge from the explorer
    edge = topods.Edge(explorer.Current())
    print(edge)

    # Create the explorer to access the vertices of the edge
    vertex_explorer = TopExp_Explorer(edge, TopAbs_VERTEX)
    # Create an empty list to store the vertices
    vertices = []

    # Loop through all the vertices connected to the edge
    while vertex_explorer.More():
        vertex = topods.Vertex(vertex_explorer.Current())
        vertices.append(vertex)
        print(vertex)
        vertex_explorer.Next()
    print("")
    # Move to the next edge
    explorer.Next()
print("Length of the L-shaped form:", total_length)

# Create a mesh representation of the shape
mesh = BRepMesh_IncrementalMesh(l_shaped_form, 0.1)

# Generate the mesh
mesh.Perform()
assert mesh.IsDone()


directory = os.path.split(__name__)[0]
stl_output_dir = os.path.abspath(directory)
assert os.path.isdir(stl_output_dir)

stl_file = os.path.join(stl_output_dir, "l_shaped_form.stl")

# Create the STL writer object
stl_writer = StlAPI_Writer()

# Add the L-shaped form to the STL writer
print(stl_writer.Write(mesh.Shape(), stl_file))
