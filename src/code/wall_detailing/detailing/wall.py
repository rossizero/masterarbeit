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
from OCC.Core.gp import gp_Quaternion
from quaternion.numpy_quaternion import quaternion
from OCC.Core.gp import gp_Vec, gp_Trsf
import quaternion


class Wall:
    def __init__(self, shape: TopoDS_Shape, ifc_wall_type: str, name: str = ""):
        self.name = name
        self.ifc_wall_type = ifc_wall_type
        self.length = 0.0
        self.width = 0.0
        self.height = 0.0

        self.rotation = np.quaternion(1, 0, 0, 0)
        self.translation = np.array([0, 0, 0])
        self.occ_shape = shape

        self.update_shape(shape)
        self.update_dimensions()

        self.openings = []

        self.left_connections = []
        self.right_connections = []

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

        self.translation = translation
        self.rotation = rotation

        transformation = gp_Trsf()
        translation = gp_Vec(*translation).Reversed()
        transformation.SetTranslation(translation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        self.occ_shape = shape

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
        """
        :return: the rotated and translated occ shape
        """
        # Apply the translation and rotation to our shape
        shape = self.occ_shape

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.translation))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        return shape

    def get_corners(self, relative: bool = False, inner=False):
        """
        ----------                  ----------
        |                           |
        .      ==  inner=False      |   .        == inner=True (distance = width/2)
        |                           |
        ----------                  ----------
        :param relative: coordinates of the corners in world or relative coordinates
        :param inner: see image above
        :return: two lines each represented by two positions
        """
        v = self.get_vertices(relative=True)
        x1 = min(v[:, 0])  # left
        x2 = max(v[:, 0])  # right
        y_mid = (min(v[:, 1]) + max(v[:, 1])) / 2.0  # center of walls width
        z1 = min(v[:, 2])  # bottom
        z2 = max(v[:, 2])  # top

        if inner:
            x1 += y_mid
            x2 -= y_mid

        # left corner
        p1 = np.array([x1, y_mid, z1])  # bottom left
        p2 = np.array([x1, y_mid, z2])  # top left

        # right corner
        p3 = np.array([x2, y_mid, z1])  # bottom right
        p4 = np.array([x2, y_mid, z2])  # top right

        rotation_matrix = quaternion.as_rotation_matrix(self.get_rotation())

        # TODO check if this is correct
        if not relative:
            p1 = np.dot(rotation_matrix, p1) + self.get_translation()
            p2 = np.dot(rotation_matrix, p2) + self.get_translation()
            p3 = np.dot(rotation_matrix, p3) + self.get_translation()
            p4 = np.dot(rotation_matrix, p4) + self.get_translation()

        return p2, p1, p4, p3

    def get_vertices(self, relative: bool = False) -> np.array:
        """
        :param relative: whether vertex positions are in world or relative coordinates
        :return: list of 3D Points as np.array
        """
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
        """
        returns the rotation part of this walls transformation
        """
        return self.rotation.copy()

    def get_translation(self) -> np.array:
        """
        returns the translation part of this walls transformation
        """
        return self.translation.copy()

    def get_location(self, x_offset: float = 0.0, y_offset: float = 0.0, z_offset: float = 0.0) -> np.array:
        """
        returns the world coordinates of this wall
        optionally accepts offsets to this walls translation before rotating the point
        """
        translation = self.get_translation()
        position = np.array([x_offset, y_offset, z_offset])
        vec = quaternion.rotate_vectors(self.get_rotation(), position)
        vec += translation
        return vec

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

    def rotate_around(self, rotation: np.quaternion, pivot_point: np.array = np.array([0.0, 0.0, 0.0])):
        """
        :param rotation: rotate
        :param pivot_point: around this point
        :return:
        """
        # ...translate...
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*-pivot_point))
        shape = BRepBuilderAPI_Transform(self.get_shape(), transformation, True, True).Shape()

        # ...rotate....
        transformation = gp_Trsf()
        rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # ...then translate back...
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*pivot_point))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        self.update_shape(shape)