import ifcopenshell
import numpy as np
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.gp import gp_Trsf, gp_Vec, gp_Quaternion
from ifcopenshell import geom, util
from OCC import VERSION
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape, topods
from ifcopenshell.util import placement
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.detailing.opening import Opening

settings = geom.settings()
# global coordinates
settings.set(settings.USE_WORLD_COORDS, True)
# Specify to return pythonOCC shapes from ifcopenshell.geom.create_shape()
settings.set(settings.USE_PYTHON_OPENCASCADE, True)


print("OCC version", VERSION)
print("IfcOpenshell version", ifcopenshell.version)


class IfcImporter:
    def __init__(self, ifc_file_path):
        self.ifc_file_path = ifc_file_path
        self.ifc_file = ifcopenshell.open(self.ifc_file_path)

    def get_shape_dimensions(self, shape: TopoDS_Shape):
        """
        :param shape: the shape to get the dimensions from
        :return: the dimensions of the shape
        """
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

        vertices = np.unique(vertices, axis=0)
        x = max(vertices[:, 0]) - min(vertices[:, 0])
        y = max(vertices[:, 1]) - min(vertices[:, 1])
        z = max(vertices[:, 2]) - min(vertices[:, 2])

        dimensions = np.array(np.around([x, y, z], decimals=6))
        length = max(dimensions[0], dimensions[1])
        width = min(dimensions[0], dimensions[1])
        height = dimensions[2]
        ret = (length, width, height)
        return ret

    def get_absolute_position(self, object_placement):
        transformation_matrix = np.eye(4, dtype=np.float64)

        # Traverse the placement hierarchy
        while object_placement is not None and object_placement.is_a("IfcLocalPlacement"):
            # Extract relative placement information
            relative_placement = object_placement.RelativePlacement
            if relative_placement.is_a("IfcAxis2Placement3D"):
                translation = np.array(relative_placement.Location.Coordinates)
                #rotation_matrix = relative_placement.Axis.Orientation.Value
                # Extract rotation information
                z_axis = relative_placement.Axis.DirectionRatios
                x_axis = relative_placement.RefDirection.DirectionRatios
                y_axis = np.cross(z_axis, x_axis)

                rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

                # Update the transformation matrix with the relative placement information
                transformation_matrix[:3, 3] += translation
                transformation_matrix[:3, :3] = np.dot(rotation_matrix, transformation_matrix[:3, :3])

            # Move to the next level of placement hierarchy
            object_placement = object_placement.PlacementRelTo

        position = np.array([transformation_matrix[0][3], transformation_matrix[1][3], transformation_matrix[2][3]])
        rotation = quaternion.from_rotation_matrix(transformation_matrix[:3, :3])

        return position, rotation

    def get_walls(self):
        ifc_walls = self.ifc_file.by_type("IfcWall")
        walls = []

        for w in ifc_walls:
            shape = geom.create_shape(settings, w).geometry
            wall = Wall(shape, "TODO")

            print("wall", wall.length, wall.width, wall.height)
            walls.append(wall)

            # get openings of the wall
            # https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcDoor.htm
            # https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcOpeningElement.htm
            for void_element in w.HasOpenings:
                o = void_element.RelatedOpeningElement
                if "Body" in [rep.RepresentationIdentifier for rep in o.Representation.Representations]:
                    shape = geom.create_shape(settings, o).geometry
                    translation, rotation = self.get_absolute_position(o.ObjectPlacement)

                    # translate back to origin
                    transformation = gp_Trsf()
                    transformation.SetTranslation(gp_Vec(*translation).Reversed())
                    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

                    transformation = gp_Trsf()
                    transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
                    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

                    dimensions = self.get_shape_dimensions(shape)
                    print("opening", dimensions)
                    opening = Opening(wall, translation, rotation, dimensions)
                    wall.openings.append(opening)

        return walls
