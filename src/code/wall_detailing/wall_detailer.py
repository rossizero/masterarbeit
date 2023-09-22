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

from detailing.wall_layer_group import WallLayerGroup
from masonry.bond import StrechedBond, GothicBond
from masonry.brick import BrickInformation, Brick
from detailing.wall import Wall
from masonry.Corner import Corner, Corners


class WallDetailer:
    def __init__(self, walls: List[Wall], brick_information: Dict[str, List[BrickInformation]]):
        self.walls = walls
        self.brick_information = brick_information

    def combine_layer_groups(self, wall_layer_groups: List[WallLayerGroup]) -> List[WallLayerGroup]:
        """
        Combines all WallLayerGroups that touch, have the same z orientation and are at 0 / 180 degrees to each other
        or are exactly above / beyond each other
        :param wall_layer_groups:
        :return:
        """
        groups = wall_layer_groups.copy()
        ret = []

        combined = False
        while len(groups) > 0:
            if not combined:
                curr = groups.pop(0)
                ret.append(curr)

            combined = False
            for g in groups:
                combined = curr.combine(g)
                if combined:
                    groups.remove(g)
                    break
        return ret

    def detail_wall_new(self, wall: WallLayerGroup):
        brick_ret = []
        original_rotation = wall.get_rotation()
        module = wall.module
        bond = StrechedBond(module)
        width = module.width  # TODO bond width

        counter = 0
        original_translation = wall.get_translation()

        for layers in wall.get_sorted_layers():
            for layer in layers:
                dimensions = np.array([layer.length, width, module.height])

                fill_left = len(layer.left_connections) == 0
                fill_right = len(layer.right_connections) == 0
                transformations = bond.apply(*dimensions, fill_left, fill_right, counter, layer.relative_x_offset)
                for tf in transformations:
                    local_position = tf.get_position()  # position in wall itself (reference point is bottom left corner)
                    local_rotation = tf.get_rotation()  # rotation of the brick around itself

                    b = Brick(tf.module)
                    b.rotate(local_rotation)
                    # need to substract half of dimensions since its position coordinates are at its center
                    center = layer.translation.copy() - dimensions / 2.0
                    local_position += center
                    local_position[2] = center[2] - module.height / 2.0
                    b.translate(local_position)
                    b.rotate_around(original_rotation)  # rotate to fit wall rotation
                    b.translate(original_translation)  # translate to wall
                    brick_ret.append(b)
            counter += 1
        return brick_ret

    def detail_corner(self, corner: Corner, bricks: List[BrickInformation]):
        brick_ret = []
        wall = corner.get_main_wall()

        # get "biggest" brick TODO maybe there is a better criteria?
        bricks.sort(key=lambda x: x.volume(), reverse=True)

        module = bricks[0]
        bond = StrechedBond(module)  # TODO must be set somewhere else
        dimensions = np.array([wall.length, wall.width, wall.height])
        transformations = bond.apply_corner(corner.line)
        original_rotation = wall.get_rotation() # quaternion.from_euler_angles(0, 0, 0) # wall1.get_rotation()# quaternion.rotate_vectors(, np.array([0.0, 0.0, 1.0]))
        corner_rotation = corner.get_rotation()

        for tf in transformations:
            local_position = tf.get_position()  # position in wall itself
            local_rotation = tf.get_rotation()  # quaternion_multiply(tf.get_rotation(), corner_rotation)  # rotation of the brick around itself
            global_position = corner.line.p1 + local_position

            b = Brick(module)
            b.rotate(local_rotation)

            # mid of two overlapping modules
            vec = np.array([module.width / 2, module.width / 2, 0.0])
            b.rotate_around(corner_rotation, vec)

            p1_rotated = quaternion.rotate_vectors(original_rotation.inverse(), corner.line.p1)

            diff = wall.get_translation() - dimensions / 2.0 - p1_rotated
            tmp = local_position + p1_rotated - vec
            mid = np.array([module.length/2, module.width/2, module.height/2])

            b.translate(tmp)
            b.rotate_around(original_rotation)

            brick_ret.append(b)
        return brick_ret

    def check_corners_new(self, wall_layer_groups: List[WallLayerGroup]) -> Corners:
        counter = 0
        corners = Corners()

        for i, w1 in enumerate(wall_layer_groups):
            for j in range(i+1, len(wall_layer_groups)):
                w2 = wall_layer_groups[j]

                r1 = w1.get_rotation()
                r2 = w2.get_rotation()
                diff = r2 * r1.inverse()
                angle = round(diff.angle(), 6)

                # check if rotation of wall leads to parallel corners
                z_part1 = quaternion.rotate_vectors(r1, np.array([0.0, 0.0, 1.0]))
                z_part2 = quaternion.rotate_vectors(r2, np.array([0.0, 0.0, 1.0]))
                z_parallel = np.isclose(abs(np.dot(z_part1, z_part2)), 1.0)
                degree90 = (angle == round(math.pi / 2, 6) or angle == round(math.pi * 1.5, 6))
                touching = w1.is_touching(w2)
                print(w1.name, "touching" if touching else "not touching ", w2.name, z_parallel, degree90)
                if not (z_parallel and degree90 and touching):
                    continue

                for l1 in w1.layers:
                    for l2 in w2.layers:
                        mid1 = l1.center
                        mid2 = l2.center

                        direction1 = quaternion.rotate_vectors(r1, np.array([1, 0, 0]))
                        direction2 = quaternion.rotate_vectors(r2, np.array([1, 0, 0]))

                        A = np.vstack((direction1, -direction2, [1, 1, 1])).T
                        b_bottom = mid2 - mid1
                        try:
                            t = np.linalg.solve(A, b_bottom)

                            # Calculate the intersection points on both lines
                            intersection_point1 = mid1 + t[0] * direction1
                            intersection_point2 = mid2 + t[1] * direction2
                            assert np.allclose(intersection_point1, intersection_point2)
                            counter += 1

                            c = Corner(intersection_point1, intersection_point2)
                            c.walls.update([l1, l2])
                            corners.add_corner(c)

                            if np.linalg.norm(intersection_point1 - l1.left_edge) < w1.module.width:  # TODO use wall width!
                                l1.left_connections.append(l2)
                            elif np.linalg.norm(intersection_point1 - l1.right_edge) < w1.module.width:
                                l1.right_connections.append(l2)

                            if np.linalg.norm(intersection_point1 - l2.left_edge) < w2.module.width:  # TODO use wall width!
                                l2.left_connections.append(l1)
                            elif np.linalg.norm(intersection_point1 - l2.right_edge) < w2.module.width:
                                l2.right_connections.append(l1)

                        except np.linalg.LinAlgError:
                            #print("no intersection found between", w1.name, "and", w2.name,  "even though they are touching")
                            continue
                        except AssertionError:
                            #print("intersection points are not the same for", w1.name, "and", w2.name,  "even though they are touching")
                            continue
        print("corner counter", counter)
        return corners

    def detail_new(self):
        bricks = []
        wall_layer_groups = []

        # convert walls to layergroups
        for w in self.walls:
            module = self.brick_information[w.ifc_wall_type]
            module.sort(key=lambda x: x.volume(), reverse=True)
            module = module[0]
            wall_layer_groups.append(WallLayerGroup.from_wall(w, module))

        # combine groups if possible
        wall_layer_groups = self.combine_layer_groups(wall_layer_groups)
        cs = self.check_corners_new(wall_layer_groups)

        for corner in cs.corners:
            walls = list(corner.walls)
            if len(walls) == 2:
                #bricks.extend(self.detail_corner(corner, self.brick_information[walls[0].ifc_wall_type]))
                pass
            elif len(walls) == 3:
                # t-joint MAYDO combine t-joints
                pass
            else:
                # crossing MAYDO combine two walls
                pass

        for wall in wall_layer_groups:
            bricks.extend(self.detail_wall_new(wall))
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

        if additional_shapes is not None and len(additional_shapes) > 0:
            if shape is None:
                shape = additional_shapes[0]

            for s in additional_shapes:
                shape = BRepAlgoAPI_Fuse(
                    shape,
                    s
                ).Shape()
        if shape is not None:
            mesh = BRepMesh_IncrementalMesh(shape, detail)
            mesh.Perform()
            assert mesh.IsDone()
            stl_export = StlAPI_Writer()
            print("Export to", file_path, " successful", stl_export.Write(mesh.Shape(), file_path))


def make_wall(length, width, height, position, rotation, ifc_wall_type, name=""):
    length, width = max(length, width), min(length, width)
    corner = gp_Pnt(-length/2.0, -width/2.0, -height/2.0)

    shape = BRepPrimAPI_MakeBox(corner, length, width, height).Shape()

    transformation = gp_Trsf()
    transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w))
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

    transformation = gp_Trsf()
    transformation.SetTranslation(gp_Vec(*position))
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

    wall = Wall(shape, ifc_wall_type, name)
    return wall


if __name__ == "__main__":
    an = math.pi / 2 * 1.22432
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}

    w1 = make_wall(10, 1, 5, np.array([5.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi/2), ifc_wall_type="test", name="w1")
    w11 = make_wall(10, 1, 10, np.array([5.5, 10.0, -1.0]), quaternion.from_euler_angles(0, 0, math.pi/2 + math.pi), ifc_wall_type="test", name="w11")
    w11_ = make_wall(5, 1, 10, np.array([5.5, 10.0, 9.0]), quaternion.from_euler_angles(0, 0, math.pi/2 + math.pi), ifc_wall_type="test", name="w11_")
    w11_2 = make_wall(5, 1, 10, np.array([5.5, 10.0, -11.0]), quaternion.from_euler_angles(0, 0, math.pi/2 + math.pi), ifc_wall_type="test", name="w11_2")
    w111 = make_wall(5, 1, 10, np.array([5.5, -7.5, 1.0]), quaternion.from_euler_angles(0, 0, math.pi/2), ifc_wall_type="test", name="w111")
    w2 = make_wall(10, 1, 5, np.array([10.0, 4.5, 1.0]), quaternion.from_euler_angles(0.0, 0.0, 0), ifc_wall_type="test", name="w2")
    w3 = make_wall(10, 1, 5, np.array([-5.5, 0.0, -1.0]), quaternion.from_euler_angles(0.0, 0.0, math.pi / 2), ifc_wall_type="test", name="w3")
    w4 = make_wall(10, 1, 5, np.array([0.0, -4.5, -3.0]), quaternion.from_euler_angles(0.0, 0.0, 0), ifc_wall_type="test", name="w4")
    w4_ = make_wall(4, 1, 5, np.array([0.0, -4.5, 2]), quaternion.from_euler_angles(0.0, 0.0, 0), ifc_wall_type="test", name="w4_")
    w41_ = make_wall(4, 1, 5, np.array([1.5, -2.0, 3]), quaternion.from_euler_angles(0.0, 0.0, math.pi / 2), ifc_wall_type="test", name="w41_")

    w1.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w11.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w11_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w11_2.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w111.rotate_around(quaternion.from_euler_angles(0.3, an, an))

    w2.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w3.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w4.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w4_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
    w41_.rotate_around(quaternion.from_euler_angles(0.3, an, an))

    walls = [w1, w2, w3, w4, w4_, w41_]
    #walls = [w1, w11_,w11_2, w11, w111]
    wallss = walls.copy()

    p = gp_Pnt(0.0, 0.0, 0.0)
    BRepPrimAPI_MakeBox(p, 1.0, 1.0, 1.0).Shape()

    wall_detailer = WallDetailer(wallss, brick_information)
    bb = []
    bb = wall_detailer.detail_new()

    #print("walls", len(wallss), "bricks", len(bb))
    WallDetailer.convert_to_stl([], "base.stl", additional_shapes=[w.get_shape() for w in walls])
    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[])
