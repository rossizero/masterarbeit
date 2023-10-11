from typing import List, Dict
import numpy as np
import quaternion

from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.StlAPI import StlAPI_Writer

from detailing.NewSolver import NewSolver
from detailing.wall_layer_group import WallLayerGroup
from detailing.wall_type_group import WallTypeGroup
from masonry.bond import Bond
from masonry.brick import BrickInformation, Brick
from detailing.wall import Wall
from masonry.Corner import Corn, Corns
from scenarios.scenarios import SimpleCorners, FancyCorners, SimpleCorners2, Window1, DoppelEck1, DoppelEck2_Closed, \
    SimpleOffset, DoppelEck3_Closed
from masonry import Corner
from detailing.solver import GraphSolver


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
            solver = NewSolver(cs, bond)
            solver.solve()

            for corner in cs.corners:
                layers = list(corner.layers)
                if len(layers) == 2:
                    bricks.extend(self.detail_corner(corner, bond))
                else:
                    # t-joint MAYDO combine t-joints
                    # crossing MAYDO combine two walls
                    pass

            for wall in wall_layer_groups:
                bricks.extend(self.detail_wall(wall, bond))
        return bricks

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

            transformations = bond.apply_layer(length=layer.length,
                                               width=width,
                                               fill_left=fill_left,
                                               fill_right=fill_right,
                                               layer=layer.get_layer_plan_index(),
                                               x_offset=layer.relative_x_offset(),
                                               reversed=layer.reversed)

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

        #for tf in bond.apply_corner(corner.get_corner_index() + corner.plan_offset):
        for tf in bond.apply_corner(corner.plan_offset):
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

    WallDetailer.convert_to_stl([], "base.stl", additional_shapes=[w.get_shape() for w in scenario.walls])

    wall_detailer = WallDetailer(scenario.walls, brick_information)
    bb = wall_detailer.detail()

    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[])
