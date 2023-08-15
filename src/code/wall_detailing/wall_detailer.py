import math
from typing import List, Dict
import numpy as np
import quaternion

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepGProp import brepgprop_VolumeProperties
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.GProp import GProp_GProps
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX, TopAbs_ShapeEnum
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape, topods
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Ax3, gp_Vec

from masonry.bond import BlockBond, StrechedBond, CrossBond, HeadBond, GothicBond
from masonry.brick import BrickInformation, Brick
from masonry.wall import Wall


class WallDetailer:
    def __init__(self, walls: List[Wall], brick_information: Dict[str, List[BrickInformation]]):
        self.walls = walls
        self.brick_information = brick_information

    def detail_wall(self, wall: Wall, bricks: List[BrickInformation]):
        print(wall, bricks)
        brick_ret = []
        if wall.is_cubic:
            # Apply the inverted translation and rotation to our shape to get axis aligned shape at [0, 0, 0]
            shape = wall.occ_shape.Reversed()
            transformation = gp_Trsf()
            original_rotation = np.quaternion(shape.Location().Transformation().GetRotation().W(),
                                              shape.Location().Transformation().GetRotation().X(),
                                              shape.Location().Transformation().GetRotation().Y(),
                                              shape.Location().Transformation().GetRotation().Z())
            transformation.SetRotation(shape.Location().Transformation().GetRotation().Inverted())
            shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

            original_translation = np.array([shape.Location().Transformation().TranslationPart().X(),
                                             shape.Location().Transformation().TranslationPart().Y(),
                                             shape.Location().Transformation().TranslationPart().Z()])
            transformation.SetTranslation(
                gp_Vec(shape.Location().Transformation().TranslationPart().Reversed()))
            shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

            print(shape.Location().Transformation().TranslationPart().X(),
                  shape.Location().Transformation().TranslationPart().Y(),
                  shape.Location().Transformation().TranslationPart().Z())

            print(shape.Location().Transformation().GetRotation().X(),
                  shape.Location().Transformation().GetRotation().Y(),
                  shape.Location().Transformation().GetRotation().Z(),
                  shape.Location().Transformation().GetRotation().W())
            print("og", original_rotation)
            # create a boundingbox around the shape
            bounding_box = Bnd_Box()
            brepbndlib_Add(shape, bounding_box)

            # Get the minimum and maximum coordinates of the bounding box
            xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()
            print(xmin, ymin, zmin, xmax, ymax, zmax)

            length = round(xmax - xmin, 6)
            width = round(ymax - ymin, 6)
            height = round(zmax - zmin, 6)
            length, width = max(length, width), min(length, width)
            print(length, width, height)

            # get "biggest" brick TODO maybe there is a better criteria?
            bricks.sort(key=lambda x: x.volume(), reverse=True)
            print(bricks[0].volume())
            module = bricks[0]

            print("og translation", original_translation)

            bond = GothicBond(module)  # TODO must be set somewhere else
            transformations = bond.apply(length, width, height)

            for tf in transformations:
                local_position = tf.get_position()  # position in wall itself
                local_rotation = tf.get_rotation()  # rotation of the brick around itself

                # need to substract half wall dimensions since its position coordinates are at its center
                global_position = local_position + original_translation - np.array([length/2, width/2, height/2])
                b = Brick(module, global_position, global_rotation=original_rotation, local_rotation=local_rotation)
                brick_ret.append(b)
        return brick_ret

    def detail_corner(self, wall1: Wall, wall2: Wall, bricks: List[BrickInformation]):
        pass

    def detail_t_joint(self, wall1: Wall, wall2: Wall, bricks: List[BrickInformation]):
        pass

    def detail_opening(self, wall: Wall, opening, bricks: List[BrickInformation]):
        pass

    def check_walls(self):
        for i, w1 in enumerate(self.walls):
            if not w1.is_cubic:
                continue
            for j in range(i+1, len(self.walls)):
                w2 = self.walls[j]
                if not w2.is_cubic:
                    continue

                # Check if they touch each other
                dist_calculator = BRepExtrema_DistShapeShape(w1.occ_shape, w2.occ_shape)
                dist_calculator.Perform()
                t = dist_calculator.IsDone() and dist_calculator.Value() <= 0.000001
                print("touching", i, j, t)
                p1 = dist_calculator.PointOnShape1(2)
                p2 = dist_calculator.PointOnShape1(2)
                print("p1", p1.X(), p1.Y(), p1.Z())
                print("p2", p2.X(), p2.Y(), p2.Z())
                if t:
                    print("Rossi", dist_calculator.NbSolution())
                    print(dist_calculator.DumpToString())
                # print(dist_calculator.PointOnShape1(1), dist_calculator.PointOnShape2(1))

                # if yes -> at what angle?

    def detail(self) -> List[Brick]:
        bricks = []
        self.check_walls()
        for wall in self.walls:
            bricks.extend(self.detail_wall(wall, self.brick_information[wall.ifc_wall_type]))

        return bricks

    @classmethod
    def convert_to_stl(cls, bricks: [Brick], path: str, detail: float = 0.1, additional_shapes: List = None):
        import os
        file_path = os.path.abspath(path)
        bricks_copy = bricks.copy()
        print(len(bricks_copy))

        if len(bricks_copy) > 0:
            shape = bricks_copy.pop(0).get_brep_shape()

            for brick in bricks_copy:
                shape = BRepAlgoAPI_Fuse(
                    shape,
                    brick.get_brep_shape()
                ).Shape()

            if additional_shapes is not None:
                for s in additional_shapes:
                    shape = BRepAlgoAPI_Fuse(
                        shape,
                        s
                    ).Shape()

            mesh = BRepMesh_IncrementalMesh(shape, detail)
            mesh.Perform()
            assert mesh.IsDone()
            stl_export = StlAPI_Writer()
            print("Export to", file_path, " successful", stl_export.Write(mesh.Shape(), file_path))


def make_occ_box(length, width, height, position, rotation):
    corner = gp_Pnt(-length/2.0, -width/2.0, -height/2.0)

    shape = BRepPrimAPI_MakeBox(corner, length, width, height).Shape()

    transformation = gp_Trsf()
    transformation.SetTranslation(gp_Vec(*position))
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

    # numpy to occ quaternion
    rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
    transformation = gp_Trsf()
    transformation.SetRotation(rotation)
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

    gprops = GProp_GProps()

    # Check if the shape is a solid
    brepgprop_VolumeProperties(shape, gprops)
    print(gprops.Mass())
    return shape


if __name__ == "__main__":
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}
    walls = [
        Wall(make_occ_box(10, 1, 5, [0, 0, 0], quaternion.from_euler_angles(0, 0, math.pi/2)), ifc_wall_type="test"),
    ]
    walls.extend(
        [
            Wall(make_occ_box(10, 1, 5, [10, 0, 0], quaternion.from_euler_angles(0, 0, math.pi / 2)),
                 ifc_wall_type="test"),
            Wall(make_occ_box(10, 1, 5, [4.5, -5.5, 0], quaternion.from_euler_angles(0, 0, 0)), ifc_wall_type="test"),
        ]
    )
    wall_detailer = WallDetailer(walls, brick_information)
    bb = wall_detailer.detail()

    # WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[w.occ_shape for w in walls])
