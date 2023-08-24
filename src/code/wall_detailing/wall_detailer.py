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
        brick_ret = []
        if wall.is_cubic():
            original_rotation = wall.get_rotation()
            original_translation = wall.get_translation()
            dimensions = np.array([wall.length, wall.width, wall.height])

            #print("og rotation", original_rotation)
            #print("og translation", original_translation)
            #print("dimensions", *dimensions)

            # get "biggest" brick TODO maybe there is a better criteria?
            bricks.sort(key=lambda x: x.volume(), reverse=True)
            #print(bricks[0].volume())
            module = bricks[0]

            bond = StrechedBond(module)  # TODO must be set somewhere else
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
        Combines two wall elements to one.
        Does not check whether they actually touch or whatever.
        Works properly for cubic walls, but I think it doesn't make much sense to use it for non cubic walls, because
        the length/width/height is being used to move the walls around.
        """
        shape = BRepAlgoAPI_Fuse(wall1.get_shape(), wall2.get_shape()).Shape()  # Fuse Operation -> rotation is set to 0
        # transformation details (it doesn't matter if we use wall1's or wall2's data)
        rotation = wall1.get_rotation()
        translation = wall1.get_translation()
        scale = np.array([wall1.length, wall1.width, wall1.height])

        # rotate backwards
        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w).Inverted())
        shape = BRepBuilderAPI_Transform(shape, transformation, True, True).Shape()

        # translate prior wall1 center to 0 0 0
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*translation).Reversed())
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # create a new wall object to retrieve the fused dimensions
        wall = Wall(shape=shape, ifc_wall_type=wall1.ifc_wall_type)

        # translate center of new wall to 0 0 0
        transformation = gp_Trsf()
        complete_translation = translation + (np.array([wall.length, wall.width, wall.height] - scale) / 2.0)
        transformation.SetTranslation(gp_Vec(*complete_translation).Reversed())
        wall.occ_shape = BRepBuilderAPI_Transform(wall.occ_shape, transformation).Shape()

        # set transformation parameters accordingly
        wall.rotation = rotation
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

                    z_part1 = quaternion.rotate_vectors(a1, np.array([0.0, 0.0, 1.0]))
                    z_part2 = quaternion.rotate_vectors(a2, np.array([0.0, 0.0, 1.0]))
                    z_parallel = np.isclose(abs(np.dot(z_part1, z_part2)), 1.0)

                    if (angle == math.pi or angle == 0.0) and z_parallel and dist_calculator.NbSolution() >= 4:
                        print("Combine", w1.name, "and", w2.name, dist_calculator.NbSolution())
                        to_combine.append((w1, w2))
                    elif (angle == round(math.pi / 2, 6) or angle == round(math.pi * 1.5, 6)) and z_parallel and dist_calculator.NbSolution() >= 4:
                        print("90 degree corner between", w1.name, "and", w2.name, dist_calculator.NbSolution())
                        x = np.array([1.0, 0.0, 0.0])
                        mid_w1 = quaternion.rotate_vectors(a1, w1.get_translation())
                        mid_w2 = quaternion.rotate_vectors(a2, w2.get_translation())
                        dir_1 = quaternion.rotate_vectors(a1, x + np.array([0.0, w1.width / 2.0, 0.0]))
                        dir_2 = quaternion.rotate_vectors(a2, x + np.array([0.0, w2.width / 2.0, 0.0]))

                        vector_between_points = mid_w2 - mid_w1
                        print(vector_between_points)
                        _ = np.cross(vector_between_points, dir_2)
                        _ = np.dot(dir_1, dir_2)
                        t = np.cross(vector_between_points, dir_2) / np.dot(dir_1, dir_2)

                        # Calculate the intersection point
                        intersection = mid_w1 + t * dir_1
                        intersection2 = mid_w2 + t * dir_2

                        print(mid_w1, mid_w2)
                        print(dir_1, dir_2)
                        print("t", np.round(np.array(t), 6))
                        print(np.round(intersection, 6))
                        print(np.round(intersection2, 6))
                        t_joint = False
                        if t_joint:
                            pass
                        else:
                            pass
                    elif z_parallel and dist_calculator.NbSolution() >= 4:
                        print("Sternchenaufgabe!!!! Edge with angle", w1.name, w2.name, math.degrees(angle))
                    else:
                        print("Touching walls", w1.name, "and", w2.name, "with angle", math.degrees(angle), "and non parallel z axis with", dist_calculator.NbSolution(), "touching points")

        
        # combines multiple wall parts or the combinations of them to a complete wall
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

        # remove all wall parts
        for w1, w2 in to_combine:
            if w1 in self.walls:
                self.walls.remove(w1)
            if w2 in self.walls:
                self.walls.remove(w2)

        # add all full walls
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


def make_wall(length, width, height, position, rotation, ifc_wall_type, name=""):
    length, width = max(length, width), min(length, width)
    corner = gp_Pnt(-length/2.0, -width/2.0, -height/2.0)

    shape = BRepPrimAPI_MakeBox(corner, length, width, height).Shape()
    wall = Wall(shape, ifc_wall_type, name)
    wall.rotation = rotation
    wall.translation = position
    return wall


if __name__ == "__main__":
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}
    w1 = make_wall(10, 1, 5, np.array([-11.0, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2), ifc_wall_type="test", name="w1")
    w4 = make_wall(10, 1, 5, np.array([4.5, -5.5, 0.0]), quaternion.from_euler_angles(0, 0, 0), ifc_wall_type="test", name="w4")
    w2 = make_wall(10, 1, 5, np.array([-21.0, 0.0, 0.0]), quaternion.from_euler_angles(0, math.pi / 2, math.pi / 2), ifc_wall_type="test", name="w2")
    w3 = make_wall(20, 1, 5, np.array([-16.0, 5.0, 0.0]), quaternion.from_euler_angles(0, math.pi / 2, math.pi / 2), ifc_wall_type="test", name="w3")
    w5 = make_wall(10, 1, 5, np.array([-31.0, 0.0, 0.0]), quaternion.from_euler_angles(0, math.pi / 2, math.pi / 2), ifc_wall_type="test", name="w5")
    w6 = make_wall(10, 1, 5, np.array([4.5, -5.5, 0.0]), quaternion.from_euler_angles(0, math.pi/2, 0), ifc_wall_type="test", name="w6")
    walls = [w1, w2, w3, w4, w5, w6]

    wallss = walls.copy()
    wall_detailer = WallDetailer(wallss, brick_information)
    bb = wall_detailer.detail()

    print("walls", len(wallss), "bricks", len(bb))
    WallDetailer.convert_to_stl([], "base.stl", additional_shapes=[w.get_shape() for w in walls])
    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[])
