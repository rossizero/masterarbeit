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
from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Quaternion, gp_Mat
from quaternion.numpy_quaternion import quaternion
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir, gp_Ax1, gp_Ax3, gp_Trsf


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
        self.ifc_wall_type = ifc_wall_type
        self.update_shape(shape)
        self.update_dimensions()

        self.openings = []

    def update_dimensions(self):
        dimensions = self._get_dimensions()
        self.length = max(dimensions[0], dimensions[1])
        self.width = min(dimensions[0], dimensions[1])
        self.height = dimensions[2]

    def update_shape(self, shape: TopoDS_Shape):
        rotation = np.quaternion(shape.Location().Transformation().GetRotation().W(),
                                      shape.Location().Transformation().GetRotation().X(),
                                      shape.Location().Transformation().GetRotation().Y(),
                                      shape.Location().Transformation().GetRotation().Z())

        translation = np.array([shape.Location().Transformation().TranslationPart().X(),
                                     shape.Location().Transformation().TranslationPart().Y(),
                                     shape.Location().Transformation().TranslationPart().Z()])

        transformation = gp_Trsf()
        rotation = rotation
        transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
        self.occ_shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

        transformation = gp_Trsf()
        translation = gp_Vec(*translation).Reversed()
        transformation.SetTranslation(translation)
        self.occ_shape = BRepBuilderAPI_Transform(self.occ_shape, transformation).Shape()


    def _get_dimensions(self) -> np.array:
        """
        returns dimensions of the not rotated objects boundingbox of the shape
        """
        v = self.get_vertices(True)
        x = max(v[:, 0]) - min(v[:, 0])
        y = max(v[:, 1]) - min(v[:, 1])
        z = max(v[:, 2]) - min(v[:, 2])
        return np.array(np.around([x, y, z], decimals=6))

    def get_shape(self) -> TopoDS_Shape:
        # Apply the translation and rotation to our shape

        transformation = gp_Trsf()
        translation = gp_Vec(*self.translation)
        transformation.SetTranslation(translation)
        shape = BRepBuilderAPI_Transform(self.occ_shape, transformation, True, True).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        return shape

    def get_vertices(self, relative: bool = False):
        shape = self.occ_shape
        if not relative:
            shape = self.get_shape()

        edge_explorer = TopExp_Explorer(shape, TopAbs_EDGE)
        vertices = []
        while edge_explorer.More():
            edge = topods.Edge(edge_explorer.Current())
            vertex_explorer = TopExp_Explorer(edge, TopAbs_VERTEX)
            while vertex_explorer.More():
                vertex = topods.Vertex(vertex_explorer.Current())
                vertex_point = BRep_Tool.Pnt(vertex)
                vertices.append(np.array([vertex_point.X(), vertex_point.Y(), vertex_point.Z()]))
                vertex_explorer.Next()
            edge_explorer.Next()

        return np.unique(vertices, axis=0)

    def get_rotation(self) -> quaternion:
        return self.rotation.copy()

    def get_translation(self) -> np.array:
        return self.translation.copy()

    def is_cubic(self) -> bool:
        """
        Compares properties of self.occ_shape to the properties of its bounding box
        if min and max vertex coordinates are equal to those of the bounding box after rotating self.occ_shape
        to align with axis -> the shape is a cube
        if volumes are equal -> there are no holes in the shape (and it's even more likely to be a cubix shape)
        :return: whether this wall is cubic
        """

        # create a boundingbox around the shape
        bounding_box = Bnd_Box()
        brepbndlib_Add(self.occ_shape, bounding_box)

        # Get the minimum and maximum coordinates of the bounding box
        xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()
        min_max_array_bbox = np.around(np.array(bounding_box.Get()), decimals=6)

        # Calculate the dimensions of the rotated box
        length = xmax - xmin
        width = ymax - ymin
        height = zmax - zmin

        # get relative coordinates of vertices
        v = self.get_vertices(True)
        coords = [
            v[:, 0], v[:, 1], v[:, 2]
        ]

        gprops = GProp_GProps()
        brepgprop_VolumeProperties(self.occ_shape, gprops)

        # check if boundingbox has the same vertices as our axis aligned shape
        # and if their volumes are equal (if not there are openings inside the shape)
        min_max_array_wall = np.around(np.array([min(coords[0]), min(coords[1]), min(coords[2]),
                                       max(coords[0]), max(coords[1]), max(coords[2])]), decimals=6)
        close = np.allclose(min_max_array_wall, min_max_array_bbox) or np.allclose(min_max_array_bbox, min_max_array_wall)
        return close and np.isclose(gprops.Mass(), length * width * height)
