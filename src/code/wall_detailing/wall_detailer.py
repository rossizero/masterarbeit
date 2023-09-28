import math
from copy import deepcopy
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
from detailing.wall_type_group import WallTypeGroup
from masonry.bond import StrechedBond, GothicBond, Bond
from masonry.brick import BrickInformation, Brick
from detailing.wall import Wall
from masonry.Corner import Corn, Corns
from scenarios.scenarios import SimpleCorners, FancyCorners, SimpleCorners2
from wall_detailing.masonry import Corner


class WallDetailer:
    def __init__(self, walls: List[Wall], brick_information: Dict[str, List[BrickInformation]]):
        self.walls = walls
        self.brick_informations = brick_information

    def detail(self):
        bricks = []
        wall_type_groups: Dict[str, WallTypeGroup] = {}

        # convert walls to layergroups
        for w in self.walls:
            if w.ifc_wall_type not in wall_type_groups.keys():
                wall_type_groups[w.ifc_wall_type] = WallTypeGroup(w.ifc_wall_type, self.brick_informations[w.ifc_wall_type])
            layer_group = WallLayerGroup.from_wall(w, wall_type_groups[w.ifc_wall_type].module)
            wall_type_groups[w.ifc_wall_type].layer_groups.append(layer_group)

        for group in wall_type_groups.values():
            # combine layer_groups if possible
            wall_layer_groups = self.combine_layer_groups(group.layer_groups)
            cs = Corner.check_for_corners(wall_layer_groups)
            bond = group.bond
            self.brute_force(group, cs, bond)

            for corner in cs.corners:
                layers = list(corner.layers)
                if len(layers) == 2:
                    #pass
                    bricks.extend(self.detail_corner(corner, bond))
                else:
                    # t-joint MAYDO combine t-joints
                    # crossing MAYDO combine two walls
                    pass

            for wall in wall_layer_groups:
                bricks.extend(self.detail_wall(wall, bond))
        return bricks

    def tmp(self, corner: Corn, bond: Bond):
        main_layer = corner.get_main_layer()
        angle = corner.get_rotation()
        ret = 0

        for og_layer in corner.layers:
            layer = deepcopy(og_layer)
            relative_rotation = (layer.parent.get_rotation() * main_layer.parent.get_rotation().inverse())
            a = relative_rotation.angle()  # we know the relative_rotation represents the z-rotation difference
            relative_rotation = quaternion.from_euler_angles(0, 0, a) * angle
            # how far the corner stretches into the layer (x direction)
            corner_length = bond.get_corner_length(layer.get_layer_index(), relative_rotation)
            rel_x_offset = layer.relative_x_offset
            moved_dist = layer.move_edge(corner.point, corner_length)
            is_left = np.linalg.norm(corner.point - layer.left_edge) < np.linalg.norm(corner.point - layer.right_edge)
            leftover_left, leftover_right = bond.leftover_of_layer(layer.length, layer.get_layer_index(), rel_x_offset, layer.parent.id)
            #leftover_left2, leftover_right2 = bond.leftover_of_layer(og_layer.length, og_layer.get_layer_index(), og_layer.relative_x_offset, og_layer.parent.id)

            c = leftover_left if len(layer.left_connections) > 0 else 0
            d = leftover_right if len(layer.right_connections) > 0 else 0

            wall = layer.parent.id
            print(layer.parent.id, is_left, c, d, c + d, "y: ", corner.point[2], rel_x_offset, layer.relative_x_offset, "dist", moved_dist, layer.length)
            ret += c + d
        print("")
        return ret

    def brute_force(self, group: WallTypeGroup, corners: Corns, bond: Bond):
        dic = {}
        for corner in corners.corners:
            walls = [layer.parent.id for layer in corner.layers]
            key = tuple(sorted(walls))
            if key not in dic.keys():
                dic[key] = []
            dic[key].append(corner)

        val = 0
        print(len(corners.corners))
        for corner in corners.corners:
            val += self.tmp(corner, bond)
        print(val)

    def combine_layer_groups(self, wall_layer_groups: List[WallLayerGroup]) -> List[WallLayerGroup]:
        """
        Combines all WallLayerGroups that touch, have the same z orientation and are at 0 / 180 degrees to each other
        or are exactly above / beyond each other
        :param wall_layer_groups: List of WallLayerGroups we want to check for combination possibilities
        :return: a new possibly smaller list of WallLayerGroups. Input WallLayerGroup Object will be altered.
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

    def detail_wall(self, wall: WallLayerGroup, bond: Bond) -> List[Brick]:
        """
        Fills a WallLayerGroup with bricks using the set bond
        :param wall: WallLayerGroup we wan to be filled
        :param bond: Bond we want to use
        :return: List of brick objects
        """
        brick_ret = []
        original_rotation = wall.get_rotation()
        module = wall.module
        width = module.width  # TODO bond width

        original_translation = wall.get_translation()

        for layer in wall.layers:
            dimensions = np.array([layer.length, width, module.height])

            fill_left = len(layer.left_connections) == 0
            fill_right = len(layer.right_connections) == 0
            transformations = bond.apply(*dimensions, fill_left, fill_right, layer.get_layer_index(), layer.relative_x_offset)

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
        return brick_ret

    def detail_corner(self, corner: Corn, bond: Bond) -> List[Brick]:
        """
        Fills a Corner with bricks using the set bond
        :param corner: Corner we want to be filled
        :param bond: Bond we want to use
        :return: List of brick objects
        """
        brick_ret = []
        main_layer = corner.get_main_layer()
        module = main_layer.parent.module

        dimensions = np.array([main_layer.length, module.width, module.height])
        original_rotation = main_layer.parent.get_rotation()
        corner_rotation = corner.get_rotation()
        layer_index = main_layer.get_layer_index()

        for tf in bond.apply_corner(layer_index):
            local_position = tf.get_position()  # position in wall itself
            local_position[2] = 0.0  # MAYDO ugly
            local_rotation = tf.get_rotation()

            b = Brick(module)
            b.rotate(local_rotation)

            # rotate around the mid of two overlapping modules
            vec = np.array([module.width / 2, module.width / 2, 0.0])
            b.rotate_around(corner_rotation, vec)

            p1_rotated = quaternion.rotate_vectors(original_rotation.inverse(), corner.point)
            p1_rotated[2] -= module.height
            tmp = local_position + p1_rotated - vec

            b.translate(tmp)
            b.rotate_around(original_rotation)

            brick_ret.append(b)

        angle = corner.get_rotation()

        for layer in corner.layers:
            relative_rotation = (layer.parent.get_rotation() * main_layer.parent.get_rotation().inverse())
            a = relative_rotation.angle()  # we know the relative_rotation represents the z-rotation difference
            relative_rotation = quaternion.from_euler_angles(0, 0, a) * angle
            # how far the corner stretches into the layer (x direction)
            corner_length = bond.get_corner_length(layer.get_layer_index(), relative_rotation)
            layer.move_edge(corner.point, corner_length)

        return brick_ret

    @staticmethod
    def convert_to_stl(bricks: [Brick], path: str, detail: float = 0.1, additional_shapes: List = None):
        import os
        file_path = os.path.abspath(path)
        bricks_copy = bricks.copy()
        print("# bricks: ", len(bricks_copy))
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


if __name__ == "__main__":
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}
    scenario = SimpleCorners2()

    wall_detailer = WallDetailer(scenario.walls, brick_information)

    bb = []
    bb = wall_detailer.detail()
    WallDetailer.convert_to_stl([], "base.stl", additional_shapes=[w.get_shape() for w in scenario.walls])
    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[])
