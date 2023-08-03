import math
from typing import List, Dict
import numpy as np
import quaternion
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Ax3, gp_Vec


class BrickInformation:
    def __init__(self, width, length, height):
        self.length = length
        self.width = width
        self.height = height


class Brick(BrickInformation):
    def __init__(self, width, length, height, position: np.array, rotation: quaternion):
        super().__init__(width, length, height)
        self.position = position
        self.rotation = rotation

    def get_brep_shape(self):
        corner = gp_Pnt(self.position[0], self.position[1], self.position[2])

        # numpy to occ quaternion
        vector_part = np.array([self.rotation.x, self.rotation.y, self.rotation.z])
        rotation = gp_Quaternion(self.rotation.w, *vector_part)
        # rotate
        transformation = gp_Trsf()
        transformation.SetRotation(rotation)
        shape = BRepPrimAPI_MakeBox(corner, self.width, self.length, self.height).Shape()
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        # translate
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.position))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        return shape


class Wall:
    def __init__(self, shape: TopoDS_Shape):
        self.ifc_wall_type = None
        self.occ_shape = shape


class Opening:
    def __init__(self):
        pass


class WallDetailer:
    def __init__(self, walls: List[Wall], brick_information: Dict[str, List[BrickInformation]]):
        self.walls = walls
        self.brick_information = brick_information

    def detail_wall(self, wall: Wall, bricks: List[BrickInformation]):
        print(wall, bricks)
        # These are methods of the TopoDS_Shape class from pythonOCC
        shape_gpXYZ = wall.occ_shape.Location().Transformation().TranslationPart()
        rot = wall.occ_shape.Location().Transformation().GetRotation()
        # These are methods of the gpXYZ class from pythonOCC
        print(shape_gpXYZ.X(), shape_gpXYZ.Y(), shape_gpXYZ.Z())
        print(rot.X(), rot.Y(), rot.Z(), rot.W())

        # Compute the bounding box of the shape
        bounding_box = Bnd_Box()
        brepbndlib_Add(wall.occ_shape, bounding_box)

        # Get the minimum and maximum coordinates of the bounding box
        xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()

        # Calculate the dimensions of the box
        length = xmax - xmin
        width = ymax - ymin
        height = zmax - zmin

        print("Dimensions of the box:")
        print("Length:", length)
        print("Width:", width)
        print("Height:", height)

        return []

    def detail_corner(self, wall1: Wall, wall2: Wall, bricks: List[BrickInformation]):
        pass

    def detail_t_joint(self, wall1: Wall, wall2: Wall, bricks: List[BrickInformation]):
        pass

    def detail_opening(self, wall: Wall, opening, bricks: List[BrickInformation]):
        pass

    def detail(self) -> List[Brick]:
        bricks = []
        for wall in self.walls:
            bricks.extend(self.detail_wall(wall, self.brick_information[wall.ifc_wall_type]))

        return bricks

    @classmethod
    def convert_to_stl(cls, bricks: [Brick], path: str, detail: float = 0.1):
        import os
        file_path = os.path.abspath(path)
        bricks_copy = bricks.copy()

        if len(bricks_copy) > 0:
            shape = bricks_copy.pop(0).get_brep_shape()

            for brick in bricks_copy:
                shape = BRepAlgoAPI_Fuse(
                    shape,
                    brick.get_brep_shape()
                ).Shape()
            mesh = BRepMesh_IncrementalMesh(shape, detail)
            mesh.Perform()
            assert mesh.IsDone()
            stl_export = StlAPI_Writer()
            print("Export to", file_path, " successful", stl_export.Write(mesh.Shape(), file_path))


def make_occ_box(width, length, height, position, rotation):
    corner = gp_Pnt(position[0], position[1], position[2])

    # numpy to occ quaternion
    vector_part = np.array([rotation.x, rotation.y, rotation.z])
    rotation = gp_Quaternion(rotation.w, *vector_part)
    transformation = gp_Trsf()
    transformation.SetRotation(rotation)
    shape = BRepPrimAPI_MakeBox(corner, width, length, height).Shape()
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
    transformation = gp_Trsf()
    transformation.SetTranslation(gp_Vec(*position))
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
    return shape


if __name__ == "__main__":
    wall = Wall(make_occ_box(1, 10, 5, [1, 2, 3], quaternion.from_euler_angles(0, 0, math.pi/3.0)))
    wall.ifc_wall_type = "test"
    brick_information = {"test": [BrickInformation(1, 2, 0.5)]}
    walls = [wall]
    wall_detailer = WallDetailer(walls, brick_information)
    b = wall_detailer.detail()
    #b = [Brick(1, 2, 0.5, np.ones(shape=(3,)) * i, quaternion.from_euler_angles(math.pi * 2 / 20.0 * i, 0, 0)) for i in range(80)]
    WallDetailer.convert_to_stl(b, "output.stl")
