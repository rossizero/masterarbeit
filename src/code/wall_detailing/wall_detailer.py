import math
from typing import List, Dict, Optional
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
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Ax1, gp_Vec

from masonry.bond import BlockBond, StrechedBond, CrossBond, HeadBond, GothicBond
from masonry.brick import BrickInformation, Brick
from masonry.wall import Wall


class WallDetailer:
    def __init__(self, walls: List[Wall], brick_information: Dict[str, List[BrickInformation]]):
        self.walls = walls
        self.brick_information = brick_information

    def detail_wall(self, wall: Wall, bricks: List[BrickInformation]):
        print(wall, bricks, wall.is_cubic())
        brick_ret = []
        if wall.is_cubic():
            original_rotation = wall.get_rotation()
            original_translation = wall.get_translation()
            dimensions = np.array([wall.length, wall.width, wall.height])

            print("og rotation", original_rotation)
            print("og translation", original_translation)
            print("dimensions", *dimensions)

            # get "biggest" brick TODO maybe there is a better criteria?
            bricks.sort(key=lambda x: x.volume(), reverse=True)
            print(bricks[0].volume())
            module = bricks[0]

            bond = GothicBond(module)  # TODO must be set somewhere else
            transformations = bond.apply(*dimensions)

            for tf in transformations:
                local_position = tf.get_position()  # position in wall itself
                local_rotation = tf.get_rotation()  # rotation of the brick around itself

                # need to substract half wall dimensions since its position coordinates are at its center
                global_position = local_position + original_translation - dimensions / 2.0
                b = Brick(module, global_position, global_rotation=original_rotation, local_rotation=local_rotation)
                brick_ret.append(b)
        return brick_ret

    def detail_corner(self, wall1: Wall, wall2: Wall, bricks: List[BrickInformation]):
        pass

    def detail_t_joint(self, wall1: Wall, wall2: Wall, bricks: List[BrickInformation]):
        pass

    def detail_opening(self, wall: Wall, opening, bricks: List[BrickInformation]):
        pass

    def combine_walls(self, wall1: Wall, wall2: Wall) -> Optional[Wall]:
        """
        Combines two wall Elements to one.
        Does not check whether they actually touch or whatever
        """
        shape = BRepAlgoAPI_Fuse(wall1.get_shape(), wall2.get_shape()).Shape()
        rotation = wall1.get_rotation()
        translation = wall1.get_translation()
        scale = np.array([wall1.length, wall1.width, wall1.height])

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
        shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*translation).Reversed())
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        wall = Wall(shape=shape, ifc_wall_type=wall1.ifc_wall_type)

        # translate center of new wall to 0 0 0
        transformation = gp_Trsf()
        complete_translation = translation - (np.array([wall.length, wall.width, wall.height]) - scale) / 2.0
        translation = gp_Vec(*complete_translation).Reversed()
        transformation.SetTranslation(translation)
        wall.occ_shape = BRepBuilderAPI_Transform(wall.occ_shape, transformation).Shape()

        wall.rotation = wall1.get_rotation()
        wall.translation = complete_translation
        return wall

    def check_walls(self):
        to_combine = []
        for i, w1 in enumerate(self.walls):
            if not w1.is_cubic():
                continue
            for j in range(i+1, len(self.walls)):
                w2 = self.walls[j]
                if not w2.is_cubic():
                    continue

                # Check if they touch each other
                # TODO check what happens when they overlap -> wie können wir das raussortieren?
                dist_calculator = BRepExtrema_DistShapeShape(w1.get_shape(), w2.get_shape())
                dist_calculator.Perform()
                touching = dist_calculator.IsDone() and dist_calculator.Value() <= 1e-9
                if touching:
                    a1 = w1.get_rotation()
                    a2 = w2.get_rotation()
                    diff = a2 * a1.inverse()
                    angle = round(diff.angle(), 6)
                    print("HALLO", a1, a2, diff, diff.angle(), math.degrees(diff.angle()))
                    print(quaternion.as_rotation_vector(a1), quaternion.as_rotation_vector(a2))
                    # angle = abs(quaternion.as_rotation_vector(a1)[2] - quaternion.as_rotation_vector(a2)[2])
                    w1_vertices = w1.get_vertices(False)
                    w2_vertices = w2.get_vertices(False)
                    if angle == math.pi or angle == 0.0 and dist_calculator.NbSolution() == 4:
                        combine = True
                        for k in range(1, dist_calculator.NbSolution()+1):
                            v1 = np.array([dist_calculator.PointOnShape1(k).X(),
                                           dist_calculator.PointOnShape1(k).Y(),
                                           dist_calculator.PointOnShape1(k).Z(),
                                           ])

                            v2 = np.array([dist_calculator.PointOnShape2(k).X(),
                                           dist_calculator.PointOnShape2(k).Y(),
                                           dist_calculator.PointOnShape2(k).Z(),
                                           ])
                            if v1 not in w1_vertices or v2 not in w2_vertices:
                                combine = False
                                break
                        if combine:
                            to_combine.append((w1, w2))

                    elif angle == round(math.pi / 2, 6) or angle == round(math.pi * 1.5, 6):
                        print("90 degree edge!!!!!!!!!!!!!!!!!!!!", dist_calculator.NbSolution())
                        for k in range(1, dist_calculator.NbSolution()+1):
                            v1 = np.array([dist_calculator.PointOnShape1(k).X(),
                                           dist_calculator.PointOnShape1(k).Y(),
                                           dist_calculator.PointOnShape1(k).Z(),
                                           ])

                            v2 = np.array([dist_calculator.PointOnShape2(k).X(),
                                           dist_calculator.PointOnShape2(k).Y(),
                                           dist_calculator.PointOnShape2(k).Z(),
                                           ])
                            print(v1, v2, np.array_equal(v1, v2))
                            #print("v1", v1 in w1.vertices)
                            #print("v2", v2 in w2.vertices)

                        t_joint = False
                        if t_joint:
                            pass
                        else:
                            pass
                    else:
                        print("Sternchenaufgabe!!!! Edge with angle", math.degrees(angle))

        groups = {}
        for w1, w2 in to_combine:
            w1_ = w1
            w2_ = w2
            w1_key = (w1,)
            w2_key = (w2,)

            for key in groups.keys():
                if w1 in key:
                    w1_ = groups[key]
                    w1_key = key
                    break

            for key in groups.keys():
                if w2 in key:
                    w2_ = groups[key]
                    w2_key = key
                    break

            combi = self.combine_walls(w2_, w1_)
            if w1_key in groups.keys(): del groups[w1_key]
            if w2_key in groups.keys(): del groups[w2_key]

            l1 = list(w1_key)
            l2 = list(w2_key)
            l1.extend(l2)
            new_key = tuple(l1)
            groups[new_key] = combi

        for w1, w2 in to_combine:
            if w1 in self.walls:
                self.walls.remove(w1)
            if w2 in self.walls:
                self.walls.remove(w2)

        for val in groups.values():
            if val not in self.walls:
                self.walls.append(val)

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
        shape = None

        if len(bricks_copy) > 0:
            shape = bricks_copy.pop(0).get_brep_shape()

        if shape is not None:
            for brick in bricks_copy:
                shape = BRepAlgoAPI_Fuse(
                    shape,
                    brick.get_brep_shape()
                ).Shape()

        if additional_shapes is not None:
            if shape is None:
                shape = additional_shapes[0]

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


def make_wall(length, width, height, position, rotation, ifc_wall_type):
    length, width = max(length, width), min(length, width)
    corner = gp_Pnt(-length/2.0, -width/2.0, -height/2.0)

    shape = BRepPrimAPI_MakeBox(corner, length, width, height).Shape()
    wall = Wall(shape, ifc_wall_type)
    wall.rotation = rotation
    wall.translation = position

    gprops = GProp_GProps()

    # Check if the shape is a solid
    brepgprop_VolumeProperties(shape, gprops)
    return wall


if __name__ == "__main__":
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}
    w1 = make_wall(10, 1, 5, np.array([-11.0, 0.0, 0.0]), quaternion.from_euler_angles(0, 1.3, math.pi / 2), ifc_wall_type="test")
    w2 = make_wall(10, 1, 5, np.array([-21.0, 0.0, 0.0]), quaternion.from_euler_angles(0, 1.3, math.pi / 2), ifc_wall_type="test")
    w5 = make_wall(10, 1, 5, np.array([-31.0, 0.0, 0.0]), quaternion.from_euler_angles(0, 1.3, math.pi / 2), ifc_wall_type="test")
    w3 = make_wall(20, 1, 5, np.array([-16.0, 5.0, 0.0]), quaternion.from_euler_angles(0, 1.3, math.pi / 2), ifc_wall_type="test")
    w4 = make_wall(10, 1, 5, np.array([4.5, -5.5, 0.0]), quaternion.from_euler_angles(0, 0, 2 * math.pi), ifc_wall_type="test")
    #walls = [w1, w2, w3, w4, w5]
    walls = [w5, w2, w4]

    wallss = walls.copy()
    wall_detailer = WallDetailer(wallss, brick_information)
    bb = wall_detailer.detail()

    print("walls", len(wallss), "bricks", len(bb))
    WallDetailer.convert_to_stl([], "base.stl", additional_shapes=[w.get_shape() for w in walls])
    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[w.get_shape() for w in wallss])
