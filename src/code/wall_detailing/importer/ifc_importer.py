import math
from typing import Tuple

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
from wall_detailing.die_mathe.pythonocc_utils import get_shape_dimensions

from wall_detailing.masonry.brick import BrickInformation

print("OCC version", VERSION)
print("IfcOpenshell version", ifcopenshell.version)


class IfcImporter:
    WallDetailingPropertySetName = "wall_detailing_properties"
    WallDetailing_GRID = "grid"
    WallDetailing_BASEMODULE = "module"
    WallDetailing_BONDTYPE = "bond type"

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

    def get_detailing_information(self, ifc_product) -> Tuple[BrickInformation, str]:
        wall_detailing_information_property_set = None
        base_module = None
        bond_type = None

        for property_set in ifc_product.IsDefinedBy:
            if property_set.is_a("IfcRelDefinesByProperties"):
                if property_set.RelatingPropertyDefinition.Name == IfcImporter.WallDetailingPropertySetName:
                    wall_detailing_information_property_set = property_set.RelatingPropertyDefinition
                    break
        if wall_detailing_information_property_set is None:
            for ifc_type in ifc_product.IsTypedBy:
                for property_set in ifc_type.RelatingType.HasPropertySets:
                    if property_set.Name == IfcImporter.WallDetailingPropertySetName:
                        wall_detailing_information_property_set = property_set
                        break
        if wall_detailing_information_property_set is None:
            return base_module, bond_type

        dic = {}
        for property_single_value in wall_detailing_information_property_set.HasProperties:
            if property_single_value.is_a('IfcPropertySingleValue'):
                dic[property_single_value.Name] = None

                if property_single_value.NominalValue.is_a("IfcReal"):
                    dic[property_single_value.Name] = round(property_single_value.NominalValue.wrappedValue, 6)
                else:
                    dic[property_single_value.Name] = str(property_single_value.NominalValue.wrappedValue)
        try:
            base_module = BrickInformation(dic[IfcImporter.WallDetailing_BASEMODULE + " x"],
                                           dic[IfcImporter.WallDetailing_BASEMODULE + " y"],
                                           dic[IfcImporter.WallDetailing_BASEMODULE + " z"],
                                           grid=np.array([dic[IfcImporter.WallDetailing_GRID + " x"],
                                                          dic[IfcImporter.WallDetailing_GRID + " y"],
                                                          dic[IfcImporter.WallDetailing_GRID + " z"]]))
            bond_type = dic[IfcImporter.WallDetailing_BONDTYPE]
        finally:
            return base_module, bond_type


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

            # try to get wall detailing information
            base_module, bond_type = self.get_detailing_information(w)

            # the shape returned by the next line of code has no internal translation and rotation set
            # but it also is not located at the origin of the world coordinate system
            # that's why we need to calculate the absolute position of the wall and reverse the translation and rotation
            shape = geom.create_shape(settings, w).geometry
            translation, rotation = self.get_absolute_position(w.ObjectPlacement)
            angle = rotation.angle()
            angle = round(angle / (math.pi / 2.0)) * (math.pi / 2.0)
            # print(rotation.angle(), angle, rotation, quaternion.from_euler_angles(0, 0, angle))
            rotation = quaternion.from_euler_angles(0, 0, angle)

            # reverse the translation and rotation (the True, True parameters are important, but IDK why)
            # it took soooo much time to figure out that this is the correct way to do it
            transformation = gp_Trsf()
            transformation.SetTranslation(gp_Vec(*translation).Reversed())
            shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

            transformation = gp_Trsf()
            transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
            shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

            # move the shape so that its center is at the origin of the world coordinate system
            if base_module is None:
                base_module = BrickInformation(0.4, 0.2, 0.12, grid=np.array([0.1, 0.1, 0.12]))

            half_dim = get_shape_dimensions(shape, base_module.grid) / 2.0

            if bond_type is None:
                bond_type = "HeadBond" if half_dim[1] * 2 == 0.4 else "StretchedBond"

            transformation = gp_Trsf()
            transformation.SetTranslation(gp_Vec(*half_dim).Reversed())
            shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

            # create a wall object and set its translation and rotation manually (MAYDO: a little bit hacky)
            _wall = Wall(shape, self.wall_type, base_module=base_module, bond_type=bond_type)
            # correct the translation
            translation = translation + quaternion.rotate_vectors(rotation, half_dim)

            translation = np.array([
                round(translation[0] / base_module.grid[0]) * base_module.grid[0],
                round(translation[1] / base_module.grid[1]) * base_module.grid[1],
                round(translation[2] / base_module.grid[2]) * base_module.grid[2]
            ])
            translation = np.round(translation, decimals=6)

            _wall.translation = translation
            _wall.rotation = rotation
            wall = Wall.make_wall(_wall.length, _wall.width, _wall.height, translation,
                                  rotation,
                                  ifc_wall_type=self.wall_type, base_module=base_module, bond_type=bond_type)

            # print("wall", wall.length, wall.width, wall.height, wall.get_translation(), wall.translation, wall.get_rotation(), wall.rotation)

            walls.append(wall)
            # get openings in the wall
            for void_element in w.HasOpenings:
                o = void_element.RelatedOpeningElement
                if "Body" in [rep.RepresentationIdentifier for rep in o.Representation.Representations]:
                    try:
                        shape = geom.create_shape(settings, o).geometry
                        translation, rotation = self.get_absolute_position(o.ObjectPlacement)
                        translation = np.array([0.0, 0.0, 0.0])
                        if o.ObjectPlacement.RelativePlacement is not None:
                            location = o.ObjectPlacement.RelativePlacement.Location
                            translation = np.array(location.Coordinates)
                            translation = np.array([
                                round(translation[0] / base_module.grid[0]) * base_module.grid[0],
                                round(translation[1] / base_module.grid[1]) * base_module.grid[1],
                                round(translation[2] / base_module.grid[2]) * base_module.grid[2]
                            ])
                            translation = np.round(translation, decimals=6)

                        transformation = gp_Trsf()
                        transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
                        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
                        dimensions = get_shape_dimensions(shape)

                        dimensions[1] = wall.width
                        rotation *= wall.get_rotation().inverse()
                        opening = Opening(wall, translation, rotation, dimensions)
                        #print("  opening", opening.length, opening.width, opening.height, opening.get_position(), translation, opening.get_rotation(), rotation)
                        wall.openings.append(opening)
                    except Exception as e:
                        print(e)
                        continue
        print("ifc importer", len(walls))
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