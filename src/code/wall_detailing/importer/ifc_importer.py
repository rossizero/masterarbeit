import math

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
from ifcopenshell.express.rules.IFC4X3 import IfcUnitEnum
from ifcopenshell.util import placement
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.detailing.opening import Opening
from wall_detailing.importer.IfcOpening import IfcOpening

print("OCC version", VERSION)
print("IfcOpenshell version", ifcopenshell.version)


class IfcImporter:
    def __init__(self, ifc_file_path, wall_type="Test"):
        self.ifc_file_path = ifc_file_path
        self.ifc_file = ifcopenshell.open(self.ifc_file_path)
        self.unit_scale = 1.0  # what factor do we need to multiply the units with to get meters
        projects = self.ifc_file.by_type("IfcProject")
        self.wall_type = wall_type

        alll = IfcUnitEnum
        if len(projects) > 0: # MAYDO differentiate between projects
            project = projects[0]
            if hasattr(project, "UnitsInContext"):
                if project.UnitsInContext is not None:
                    for unit in project.UnitsInContext.Units:
                        pass


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
        ret = np.array([length, width, height])
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

                # Ensure the basis is orthonormal
                x_axis = x_axis / np.linalg.norm(x_axis)
                y_axis = y_axis / np.linalg.norm(y_axis)
                z_axis = z_axis / np.linalg.norm(z_axis)

                rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

                # Update the transformation matrix with the relative placement information
                transformation_matrix[:3, 3] += translation
                transformation_matrix[:3, :3] = np.dot(rotation_matrix, transformation_matrix[:3, :3])

            # Move to the next level of placement hierarchy
            object_placement = object_placement.PlacementRelTo

        position = np.array([transformation_matrix[0][3], transformation_matrix[1][3], transformation_matrix[2][3]])
        rotation = quaternion.from_rotation_matrix(np.around(transformation_matrix[:3, :3], decimals=6))
        #rotation = np.quaternion(round(rotation.z, 6), round(rotation.x, 6), round(rotation.y, 6), round(rotation.w, 6))
        return np.round(position, decimals=6), rotation

    def get_walls(self):
        ifc_walls = self.ifc_file.by_type("IfcWall")
        walls = []

        settings = geom.settings()
        # global coordinates
        settings.set(settings.USE_WORLD_COORDS, True)
        # Specify to return pythonOCC shapes from ifcopenshell.geom.create_shape()
        settings.set(settings.USE_PYTHON_OPENCASCADE, True)

        for w in ifc_walls:

            if True not in [rep.RepresentationType == "SweptSolid" for rep in w.Representation.Representations]:
                continue

            # the shape returned by the next line of code has no internal translation and rotation set
            # but it also is not located at the origin of the world coordinate system
            # that's why we need to calculate the absolute position of the wall and reverse the translation and rotation
            shape = geom.create_shape(settings, w).geometry
            translation, rotation = self.get_absolute_position(w.ObjectPlacement)

            # reverse the translation and rotation (the True, True parameters are important, but idk why)
            # it took soooo much time to figure out that this is the correct way to do it
            transformation = gp_Trsf()
            transformation.SetTranslation(gp_Vec(*translation).Reversed())
            shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

            transformation = gp_Trsf()
            transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
            shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

            # move the shape so that its center is at the origin of the world coordinate system
            half_dim = self.get_shape_dimensions(shape) / 2.0
            transformation = gp_Trsf()
            transformation.SetTranslation(gp_Vec(*half_dim).Reversed())
            shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

            # create a wall object and set its translation and rotation manually (MAYDO: a little bit hacky)
            wall = Wall(shape, self.wall_type)
            wall.translation = translation + np.round(quaternion.rotate_vectors(rotation, half_dim), decimals=6)
            wall.rotation = rotation

            print("wall", wall.length, wall.width, wall.height, wall.get_translation(), translation, wall.get_rotation(), rotation)
            walls.append(wall)

            # get openings of the wall
            # https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcDoor.htm
            # https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcOpeningElement.htm
            for void_element in w.HasOpenings:
                o = void_element.RelatedOpeningElement
                if "Body" in [rep.RepresentationIdentifier for rep in o.Representation.Representations]:
                    try:
                        shape = geom.create_shape(settings, o).geometry
                        translation, rotation = self.get_absolute_position(o.ObjectPlacement)
                        transformation = gp_Trsf()
                        transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
                        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

                        #transformation = gp_Trsf()
                        #transformation.SetTranslation(gp_Vec(*translation).Reversed())
                        #shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

                        dimensions = self.get_shape_dimensions(shape)
                        dimensions = (dimensions[0], wall.width, dimensions[2])
                        print("opening", dimensions, translation, rotation)
                        translation -= quaternion.rotate_vectors(wall.get_rotation().inverse(), wall.get_translation())  # wall.get_translation() -
                        #translation += np.array([wall.length / 2, wall.width / 2, wall.height / 2])
                        translation += np.array([dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2])
                        #translation += quaternion.rotate_vectors(wall.get_rotation(), np.array([dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2]))

                        #translation += quaternion.rotate_vectors(wall.get_rotation().inverse(), np.array([wall.length / 2, wall.width / 2, wall.height / 2]))
                        #translation += quaternion.rotate_vectors(wall.get_rotation(), )
                        #translation -= np.array([dimensions[0] / 2, dimensions[1] / 2, 0.0])
                        #translation -= quaternion.rotate_vectors(wall.get_rotation(), np.array([0.0, dimensions[1] / 2.0, 0.0]))
                        rotation *= wall.get_rotation().inverse()  # rotation = 0 rotation
                        #print("rotation", rotation)
                        opening = IfcOpening(wall, translation, rotation, dimensions)
                        opening2 = Opening(wall, np.array([0.0, 0.0, 0.0]), rotation, (0.6, 0.6, 0.6))
                        opening3 = IfcOpening(wall, np.array([0.0, 0.0, 0.0]), rotation, (0.4, 0.4, 0.4))
                        #wall.openings.append(opening)
                        #wall.openings.append(opening2)
                        #wall.openings.append(opening3)
                    except Exception as e:
                        print(e)
                        continue

        return walls

    def code_for_latex(self):
        ifc_walls = self.ifc_file.by_type("IfcWall")

        settings = geom.settings()
        # global coordinates
        settings.set(settings.USE_WORLD_COORDS, True)
        # Specify to return pythonOCC shapes from ifcopenshell.geom.create_shape()
        settings.set(settings.USE_PYTHON_OPENCASCADE, True)

        for ifc_wall in ifc_walls:
            shape = geom.create_shape(settings, ifc_wall).geometry
            # all placements are relative to some parent class until we reach IfcProject
            # this function returns the absolute position of the wall by traversing the placement hierarchy
            translation, rotation = self.get_absolute_position(ifc_wall.ObjectPlacement)

            # get annotated wall type from IfcProduct
            wall_type = self.get_wall_type(ifc_wall)
            wall = Wall(shape, translation, rotation, wall_type)

            # now we look for openings in the wall
            for void_element in ifc_wall.HasOpenings:
                opening = void_element.RelatedOpeningElement
                shape = geom.create_shape(settings, opening).geometry
                # check the dimensions of the shape object
                dimensions = self.get_shape_dimensions(shape)
                translation, rotation = self.get_absolute_position(opening.ObjectPlacement)

                # the opening is relative to the wall, so we need to transform its global position to a relative one
                rotation *= wall.get_rotation().inverse()
                translation -= wall.get_translation() - np.array([wall.length / 2, wall.width / 2, wall.height / 2])
                translation -= np.array([dimensions[0] / 2, dimensions[1] / 2, 0.0])
                opening = Opening(wall, translation, rotation, dimensions)
                wall.openings.append(opening)