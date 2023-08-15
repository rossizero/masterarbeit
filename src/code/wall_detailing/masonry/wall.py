from typing import List, Union, Tuple

import numpy as np

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepGProp import brepgprop_VolumeProperties
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.GProp import GProp_GProps
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape, topods
from OCC.Core.gp import gp_Trsf, gp_Vec
from quaternion.numpy_quaternion import quaternion


class Opening:
    """
    basically a box that describes a hole in a wall
    """
    def __init__(self, position: np.array, rotation: quaternion, dimensions: Tuple[float, float, float]):
        self.position = position
        self.rotation = rotation
        self.length = max(dimensions[0], dimensions[1])
        self.width = min(dimensions[0], dimensions[1])
        self.height = dimensions[2]


class Wall:
    def __init__(self, shape: TopoDS_Shape, ifc_wall_type: str):
        self.ifc_wall_type = None
        self.occ_shape = shape
        self.ifc_wall_type = ifc_wall_type
        self.openings = []
        self.is_cubic = self._is_cubic()

    def _is_cubic(self) -> bool:
        """
        Compares properties of self.occ_shape to the properties of its bounding box
        if min and max vertex coordinates are equal to those of the bounding box after rotating self.occ_shape
        to align with axis -> the shape is a cube
        if volumes are equal -> there are no holes in the shape (and it's even more likely to be a cubix shape)
        :return: whether this wall is cubic
        """
        # Apply the inverted rotation to our shape to get axis aligned shape
        rotated_box_shape = self.occ_shape.Reversed()
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(self.occ_shape.Location().Transformation().TranslationPart().Reversed()))
        rotated_box_shape = BRepBuilderAPI_Transform(rotated_box_shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(self.occ_shape.Location().Transformation().GetRotation().Inverted())
        rotated_box_shape = BRepBuilderAPI_Transform(rotated_box_shape, transformation).Shape()

        # create a boundingbox around the shape
        bounding_box = Bnd_Box()
        brepbndlib_Add(rotated_box_shape, bounding_box)

        # Get the minimum and maximum coordinates of the bounding box
        xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()
        min_max_array_bbox = np.around(np.array(bounding_box.Get()), decimals=6)

        # Calculate the dimensions of the rotated box
        length = xmax - xmin
        width = ymax - ymin
        height = zmax - zmin

        edge_explorer = TopExp_Explorer(rotated_box_shape, TopAbs_EDGE)
        coords = [[], [], []]
        while edge_explorer.More():
            edge = topods.Edge(edge_explorer.Current())
            vertex_explorer = TopExp_Explorer(edge, TopAbs_VERTEX)
            while vertex_explorer.More():
                vertex = topods.Vertex(vertex_explorer.Current())
                vertex_point = BRep_Tool.Pnt(vertex)
                coords[0].append(vertex_point.X())
                coords[1].append(vertex_point.Y())
                coords[2].append(vertex_point.Z())
                vertex_explorer.Next()
            edge_explorer.Next()

        gprops = GProp_GProps()
        brepgprop_VolumeProperties(self.occ_shape, gprops)

        # check if boundingbox has the same vertices as our axis aligned shape
        # and if their volumes are equal (if not there are openings inside the shape)
        min_max_array_wall = np.around(np.array([min(coords[0]), min(coords[1]), min(coords[2]),
                                       max(coords[0]), max(coords[1]), max(coords[2])]), decimals=6)
        close = np.allclose(min_max_array_wall, min_max_array_bbox) or np.allclose(min_max_array_bbox, min_max_array_wall)
        return close and np.isclose(gprops.Mass(), length * width * height)
