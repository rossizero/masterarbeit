import math
from typing import List, Dict, Optional
import numpy as np
import quaternion

from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec

from masonry.bond import StrechedBond, GothicBond, CrossBond
from masonry.brick import BrickInformation, Brick
from masonry.wall import Wall
from masonry.Corner import Corner, Corners, Line


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0.w, quaternion0.x, quaternion0.y, quaternion0.z
    w1, x1, y1, z1 = quaternion1.w, quaternion1.x, quaternion1.y, quaternion1.z

    return np.quaternion(-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0)


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

            # get "biggest" brick TODO maybe there is a better criteria?
            bricks.sort(key=lambda x: x.volume(), reverse=True)
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

    def detail_corner(self, corner: Corner, bricks: List[BrickInformation]):
        brick_ret = []
        wall = corner.get_main_wall()

        print("main wall", wall.name)
        # get "biggest" brick TODO maybe there is a better criteria?
        bricks.sort(key=lambda x: x.volume(), reverse=True)
        # print(bricks[0].volume())
        module = bricks[0]
        bond = StrechedBond(module)  # TODO must be set somewhere else
        dimensions = np.array([wall.length, wall.width, wall.height])
        transformations = bond.apply_corner(corner.line)
        original_rotation = wall.get_rotation() # quaternion.from_euler_angles(0, 0, 0) # wall1.get_rotation()# quaternion.rotate_vectors(, np.array([0.0, 0.0, 1.0]))
        #corner_rotation = corner.get_rotation()
        print(wall.get_translation() - dimensions / 2.0)
        print(corner.line.p1)

        for tf in transformations:
            local_position = tf.get_position()  # position in wall itself
            local_rotation = tf.get_rotation()  # quaternion_multiply(tf.get_rotation(), corner_rotation)  # rotation of the brick around itself
            global_position = corner.line.p1 + local_position

            b = Brick(module)
            b.rotate(local_rotation)

            # mid of two overlapping modules
            vec = np.array([module.width / 2, module.width / 2, 0.0])
            b.rotate_around(corner.get_rotation(), vec)

            p1_rotated = quaternion.rotate_vectors(original_rotation.inverse(), corner.line.p1)

            diff = wall.get_translation() - dimensions / 2.0 - p1_rotated
            print(diff)
            tmp = local_position + p1_rotated - vec
            mid = np.array([module.length/2, module.width/2, module.height/2])

            b.translate(tmp)
            b.rotate_around(original_rotation)
            #b.translate(-tmp)
            #b.translate(-wall.get_translation() + dimensions / 2.0)


            #b.translate(dimensions / 2.0 - wall.get_translation() + corner.line.p1)
            #b.translate(corner.line.p1)
            brick_ret.append(b)
        # reduce wall size

        return brick_ret

    def detail_t_joint(self, corner: Corner, bricks: List[BrickInformation]):
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

    def check_corners(self):
        corners = Corners()
        for i, w1 in enumerate(self.walls):
            for j in range(i+1, len(self.walls)):
                w2 = self.walls[j]

                r1 = w1.get_rotation()
                r2 = w2.get_rotation()
                diff = r2 * r1.inverse()
                angle = round(diff.angle(), 6)

                A1, B1, C1, D1 = w1.get_corners()
                A2, B2, C2, D2 = w2.get_corners()

                # check if rotation of wall leads to parallel corners
                z_part1 = quaternion.rotate_vectors(r1, np.array([0.0, 0.0, 1.0]))
                z_part2 = quaternion.rotate_vectors(r2, np.array([0.0, 0.0, 1.0]))
                z_parallel = np.isclose(abs(np.dot(z_part1, z_part2)), 1.0)

                dist_calculator = BRepExtrema_DistShapeShape(w1.get_shape(), w2.get_shape())
                dist_calculator.Perform()
                touching = dist_calculator.IsDone() and dist_calculator.Value() <= 1e-9
                degree90 = (angle == round(math.pi / 2, 6) or angle == round(math.pi * 1.5, 6))

                if not touching or not z_parallel or not degree90:
                    continue

                # TODO do for each layer -> if walls are not the same height this calculation breaks
                mid_w1_bottom = w1.get_location(z_offset=-w1.height / 2.0)
                mid_w2_bottom = w2.get_location(z_offset=-w2.height / 2.0)
                mid_w1_top = w1.get_location(z_offset=w1.height / 2.0)
                mid_w2_top = w2.get_location(z_offset=w2.height / 2.0)

                direction1 = quaternion.rotate_vectors(r1, np.array([1, 0, 0]))
                direction2 = quaternion.rotate_vectors(r2, np.array([1, 0, 0]))

                A = np.vstack((direction1, -direction2, [1, 1, 1])).T
                b_bottom = mid_w2_bottom - mid_w1_bottom
                b_top = mid_w2_top - mid_w1_top

                try:
                    t_values_bottom = np.linalg.solve(A, b_bottom)
                    t_values_top = np.linalg.solve(A, b_top)
                    # Calculate the intersection points on both lines
                    intersection_point1 = mid_w1_bottom + t_values_bottom[0] * direction1
                    intersection_point2 = mid_w2_bottom + t_values_bottom[1] * direction2

                    intersection_point3 = mid_w1_top + t_values_top[0] * direction1
                    intersection_point4 = mid_w2_top + t_values_top[1] * direction2

                    # both directions should return the same intersection point
                    assert np.allclose(intersection_point1, intersection_point2)
                    assert np.allclose(intersection_point3, intersection_point4)
                    c = Corner(intersection_point1, intersection_point3)
                    c.walls.update([w1, w2])
                    corners.add_corner(c)
                    print("found corner between", w1.name, "and", w2.name, "with angle", round(math.degrees(angle)))

                except np.linalg.LinAlgError:
                    print("no intersection found between", w1.name, "and", w2.name, "even though they are touching")
                    continue
                except AssertionError:
                    print("intersection points are not the same for", w1.name, "and", w2.name, "even though they are touching")
                    continue

                print(z_parallel)
        return corners

    def check_walls_to_combine(self):
        to_combine = []
        to_corner = []
        for i, w1 in enumerate(self.walls):
            for j in range(i+1, len(self.walls)):
                w2 = self.walls[j]

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
                        #print("90 degree corner between", w1.name, "and", w2.name, dist_calculator.NbSolution())
                        to_corner.append((w1, w2))
                    elif z_parallel and dist_calculator.NbSolution() >= 4:
                        #print("Sternchenaufgabe!!!! Edge with angle", w1.name, w2.name, math.degrees(angle))
                        pass
                    else:
                        pass
                        #print("Touching walls", w1.name, "and", w2.name, "with angle", math.degrees(angle), "and non parallel z axis with", dist_calculator.NbSolution(), "touching points")

        
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
        # remove non cubic walls
        walls = self.walls.copy()
        for wall in walls:
            if not wall.is_cubic():
                self.walls.remove(wall)
        # 1. Step look for walls that can be combined, combine them and remove the old parts from self.walls
        self.check_walls_to_combine()
        # 2. Step look at each wall "edge" and check if it is a corner or the end of a wall
        cs = self.check_corners()
        for corner in cs.corners:
            walls = list(corner.walls)
            if len(walls) == 2:
                if walls[0].ifc_wall_type == walls[1].ifc_wall_type:
                    # normal corner
                    bricks.extend(self.detail_corner(corner, self.brick_information[walls[0].ifc_wall_type]))
                else:
                    # corner with two different wall types
                    pass
            elif len(walls) == 3:
                # t-joint MAYDO combine t-joints
                pass
            else:
                # crossing MAYDO combine two walls
                pass

        for wall in self.walls:
            pass
            #bricks.extend(self.detail_wall(wall, self.brick_information[wall.ifc_wall_type]))

        return bricks

    @staticmethod
    def convert_to_stl(bricks: [Brick], path: str, detail: float = 0.1, additional_shapes: List = None):
        import os
        file_path = os.path.abspath(path)
        bricks_copy = bricks.copy()
        print(len(bricks_copy))
        shape = None

        if len(bricks_copy) > 0:
            shape = bricks_copy.pop(0).shape  # get_brep_shape()

        if shape is not None:
            for brick in bricks_copy:
                shape = BRepAlgoAPI_Fuse(
                    shape,
                    brick.shape  # get_brep_shape()
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
    an = math.pi/3
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}
    w1 = make_wall(10, 1, 5, np.array([-11.0, 0.0, 0.0]), quaternion.from_euler_angles(0.0, an, math.pi / 2), ifc_wall_type="test", name="w1")
    print(w1.get_corners(True))
    print(w1.get_corners(False))
    w4 = make_wall(10, 1, 5, np.array([4.5, -5.5, 0.0]), quaternion.from_euler_angles(0.0, an, 0.0), ifc_wall_type="test", name="w4")
    w41 = make_wall(10, 1, 5, np.array([4.5, -16.5, 0.0]), quaternion.from_euler_angles(0.0, an, 0.0), ifc_wall_type="test", name="w41")
    w42 = make_wall(10, 1, 5, np.array([-11.0, -9.0, 0.0]), quaternion.from_euler_angles(0.0, an,  math.pi / 2), ifc_wall_type="test", name="w42")
    w2 = make_wall(10, 1, 5, np.array([-21.0, 0.0, 0.0]), quaternion.from_euler_angles(0, math.pi / 2, math.pi / 2), ifc_wall_type="test", name="w2")
    w3 = make_wall(20, 1, 5, np.array([-16.0, 5.0, 0.0]), quaternion.from_euler_angles(0, math.pi / 2, math.pi / 2), ifc_wall_type="test", name="w3")
    w5 = make_wall(10, 1, 5, np.array([-31.0, 0.0, 0.0]), quaternion.from_euler_angles(0, math.pi / 2, math.pi / 2), ifc_wall_type="test", name="w5")
    w6 = make_wall(4, 1, 5, np.array([-2.0, 0.0, 0.0]), quaternion.from_euler_angles(0.3, 0.2, math.pi / 3), ifc_wall_type="test", name="w6")
    #walls = [w1, w2, w3, w4, w5, w6]
    walls = [w1, w4, w6, w41, w42]
    #walls = [w1]
    wallss = walls.copy()
    wall_detailer = WallDetailer(wallss, brick_information)
    bb = wall_detailer.detail()
    bricks = brick_information["test"]
    bricks.sort(key=lambda x: x.volume(), reverse=True)
    # print(bricks[0].volume())
    module = bricks[0]
    b = Brick(module, np.array([0.0,0.0,0.0]), global_rotation=np.quaternion(1,0,0,0), local_rotation=np.quaternion(1,0,0,0))
    b.translate(np.array([2.0, 0.0, 0.0]))
    b.rotate(quaternion.from_euler_angles(0, 0, math.pi/2))
    b.rotate_around(quaternion.from_euler_angles(0, 0, math.pi/2))
    print("walls", len(wallss), "bricks", len(bb))
    WallDetailer.convert_to_stl([], "base.stl", additional_shapes=[w.get_shape() for w in walls])
    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[b.shape])
