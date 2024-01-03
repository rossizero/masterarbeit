import os
from typing import List, Dict
import numpy as np
import quaternion
from OCC.Core import BRepAlgoAPI, TopTools

from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.StlAPI import StlAPI_Writer

from detailing.layered_solver import LayeredSolver
from detailing.wall_layer_group import WallLayerGroup
from detailing.wall_type_group import WallTypeGroup
from masonry.bond.abstract_bond import Bond

# --------------------------------------------- #
# IMPORTANT needed to import all bonds, so they can be used in the ifc importer
from masonry.bond.cross_bond import CrossBond
from masonry.bond.head_bond import HeadBond
from masonry.bond.stretched_bond import StretchedBond
from masonry.bond.gothic_bond import GothicBond
from masonry.bond.block_bond import BlockBond
# --------------------------------------------- #

from masonry.brick import BrickInformation, Brick
from detailing.wall import Wall
from masonry.corner_rep import Corn, Corns
from scenarios.scenarios import SimpleCorners, FancyCorners, SimpleCorners2, Window1, DoppelEck1, DoppelEck2_Closed, \
    SimpleOffset, DoppelEck3_Closed, SmallWall, TJoint1, Bug1, DoppelEck2_Closed_TJoint, ThickWall, ThickWallAllCorners, \
    OverlappingWalls, LucaScenario, EmptyScenario, LucaWaende_duenn, LucaWaende_dick
from masonry import corner_rep
from wall_detailing.exporter.BrickExporter import BrickExporter
from wall_detailing.exporter.BrickToOntologie import BrickToOntology
from wall_detailing.importer.ifc_importer import IfcImporter
from wall_detailing.masonry import brick
from scenarios.scenarios_for_text.CombinationExample import CombinationExampleForText
from scenarios.scenarios_for_text.SimpleWallEndings import Single_Wall_Slim, Single_Wall_Thick
from wall_detailing.scenarios.scenarios_for_text.DifferentBonds import BasicsStretchedBond, BasicsCrossBond, \
    BasicsGothicBond, BasicsHeadBond
from wall_detailing.scenarios.scenarios_for_text.ExportSample import RealisationExportScenario
from wall_detailing.scenarios.scenarios_for_text.SimpleCorner import SimpleCorner
from wall_detailing.scenarios.scenarios_for_text.scenario1 import Scenario1
from wall_detailing.scenarios.scenarios_for_text.scenario2 import Scenario2
from wall_detailing.scenarios.scenarios_for_text.scenario4_ontology import Scenario4_Ontology
from wall_detailing.scenarios.scenarios_for_text.small_test import SmallTestToCompareIFC
from wall_detailing.scenarios.ifc_scenario import IFCScenario


class WallDetailer:
    def __init__(self, walls: List[Wall]):
        self.walls = walls

    def detail(self):
        """
        Actual detailing routine described in the latex files. This is the main entry point for the detailing process.
        """
        bricks = []
        wall_type_groups: Dict[str, WallTypeGroup] = {}

        # convert walls to layergroups
        print("grouping walls now")
        for w in self.walls:
            k = str(w.detailing_information)

            if not w.detailing_information.base_module.is_valid():
                continue

            if k not in wall_type_groups.keys():
                wall_type_groups[k] = WallTypeGroup(w.detailing_information)

            layer_group = WallLayerGroup.from_wall(w, wall_type_groups[k].module)
            wall_type_groups[k].layer_groups.append(layer_group)

        print("detailing now")
        for group in wall_type_groups.values():
            # combine layer_groups if possible
            wall_num = len(group.layer_groups)
            wall_layer_groups = self.combine_layer_groups(group.layer_groups)
            print("combined", wall_num, "walls to", len(wall_layer_groups), "walls")

            for wall in wall_layer_groups:
                wall.apply_openings()

            print("applied wall openings, now checking corners")
            cs = corner_rep.check_for_corners(wall_layer_groups)

            print("created corners, now solving them")
            bond = group.bond
            solver = LayeredSolver(cs, bond)
            solver.solve()

            print("now starting to calculate bricks")
            for corner in cs.corners:
                layers = list(corner.layers)
                if len(layers) == 2:
                    bricks.extend(self.detail_corner(corner, bond))
                else:
                    # t-joint MAYDO combine t-joints
                    # crossing MAYDO combine two walls
                    pass

            for wall in wall_layer_groups:
                # wall.apply_openings()
                bricks.extend(self.detail_wall(wall, bond))
                bricks.extend(wall.get_opening_lintels())
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

        original_translation = wall.get_translation()
        for layer in wall.get_sorted_layers(grouped=False):
            dimensions = np.array([layer.length, wall.wall.width, module.height])

            fill_left = len(layer.left_connections) == 0 or force_fill_holes
            fill_right = len(layer.right_connections) == 0 or force_fill_holes

            transformations = bond.apply_layer(length=layer.length,
                                               width=wall.wall.width,
                                               fill_left=fill_left,
                                               fill_right=fill_right,
                                               layer=layer.get_layer_plan_index(),
                                               x_offset=layer.relative_x_offset(),
                                               reversed=layer.parent.reversed)

            for tf in transformations:
                local_position = tf.get_position()  # position in wall itself (reference point is bottom left corner)
                local_rotation = tf.get_rotation()  # rotation of the brick around itself

                b = Brick(tf.module)
                b.rotate(local_rotation)
                # need to substract half of dimensions since its position coordinates are at its center
                center = layer.translation.copy() - dimensions / 2.0
                local_position += center
                local_position[2] = center[2]# - module.height / 2.0
                b.translate(local_position)
                b.rotate_around(original_rotation)  # rotate to fit wall rotation
                b.translate(original_translation)  # translate to wall
                brick_ret.append(b)
        return brick_ret

    def detail_corner(self, corner: Corn, bond: Bond) -> List[Brick]:
        """
        Fills a Corner with bricks using the given bond
        :param corner: Corner we want to be filled
        :param bond: Bond we want to use
        :return: List of brick objects
        """
        brick_ret = []

        main_layer = corner.get_main_layer()
        module = main_layer.parent.module
        original_rotation = main_layer.parent.get_rotation()
        corner_rotation = corner.get_rotation()
        width = main_layer.parent.wall.width

        for tf in bond.apply_corner(corner.plan_offset):
            local_position = tf.get_position()  # position in wall itself
            local_position[2] = 0.0  # MAYDO ugly
            local_rotation = tf.get_rotation()

            b = Brick(tf.module)
            b.rotate(local_rotation)

            # rotate around the mid of two overlapping modules
            b.rotate_around(corner_rotation, -tf.translation.offset)

            point = corner.point
            p1_rotated = quaternion.rotate_vectors(original_rotation.inverse(), point)
            p1_rotated[2] -= module.height / 2.0

            tmp = local_position + p1_rotated - quaternion.rotate_vectors(corner_rotation, np.array([width / 2.0, width / 2.0, 0]))

            b.translate(tmp)
            b.rotate_around(original_rotation)
            brick_ret.append(b)

        return brick_ret

    @staticmethod
    def convert_to_stl(bricks: [Brick], path: str, detail: float = 0.1, additional_shapes: List = []):
        import os
        file_path = os.path.abspath(path)
        bricks_copy = bricks.copy()

        if len(bricks_copy) + len(additional_shapes) > 0:
            args = TopTools.TopTools_ListOfShape()  # whatever

            for brick in bricks_copy:
                args.Append(brick.shape)

            for shape in additional_shapes:
                args.Append(shape)

            # since this is c++ backend we need to hold some objects in memory to make stuff work
            fop = BRepAlgoAPI.BRepAlgoAPI_Fuse()
            fop.SetRunParallel(True)
            # idk why we need to set both arguments and tools
            fop.SetArguments(args)
            fop.SetTools(args)
            build = fop.Build()  # for example this one

            shape = fop.Shape()

            if shape is None or not fop.IsDone():
                print("[ERROR] Fusion failed", shape, fop.IsDone())
            else:
                mesh = BRepMesh_IncrementalMesh(shape, detail)
                mesh.Perform()
                assert mesh.IsDone()

                stl_export = StlAPI_Writer()
                print("Export to", file_path, " successful", stl_export.Write(mesh.Shape(), file_path),
                      "num bricks:", len(bricks_copy))
                return True
        return False


def building_plan_to_stl(plan: List[Brick], until_step: int = -1):
    if until_step == -1:
        until_step = len(plan)
    n = output_dir + "output_" + str(until_step) + ".stl"
    if until_step < len(plan):
        WallDetailer.convert_to_stl(plan[:until_step], n)


#############################################
#                 SETTINGS                  #
#############################################
use_ontology = True
deduct_building_plan = False  # this takes quite long so we can disable it if not needed
export_partly_built_stls = [6, 15, 23]
calculate_neighbors = True
force_fill_holes = False  # somtimes useful when using gothic bond

export_base_stl = True  # tip: to look at stl files either use blender or the windows 3d viewer
export_openings_stl = True
export_solution_json = True
export_solution_stl = True

output_dir = "output/"
#############################################


if __name__ == "__main__":
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print("available bonds", Bond.BondTypes.keys())

    # ifc scenarios of the latex project
    scenario = IFCScenario("../../models/scenarios/Scenario1/scenario1_tower_thick_walls_headbond.ifc")
    scenario = IFCScenario("../../models/scenarios/Scenario1/scenario1_tower_thick_walls_crossbond.ifc")
    scenario = IFCScenario("../../models/scenarios/Scenario1/scenario1_tower_thin_walls.ifc")
    scenario = IFCScenario("../../models/scenarios/Scenario2/scenario2.ifc")
    scenario = IFCScenario("../../models/scenarios/Scenario3/scenario3.ifc")
    scenario = IFCScenario("../../models/scenarios/Scenario1/scenario1_tower_thin_walls.ifc")

    # rebuilt scenarios of the latex project
    scenario = Scenario1()
    scenario = Scenario2()
    scenario = Scenario4_Ontology()

    # some scenarios for the figures in the latex project
    scenario = CombinationExampleForText()  # chapters 5.1.4 and 6.2.3
    scenario = SimpleCorner()  # chapter 5.1.4
    scenario = BasicsStretchedBond()  # chapter 4.5.1
    scenario = SmallTestToCompareIFC()  # just a small test
    scenario = BasicsCrossBond()  # chapter 4.5.1
    scenario = BasicsGothicBond()  # chapter 4.5.1
    scenario = BasicsHeadBond()  # chapter 4.5.1
    scenario = RealisationExportScenario()  # chapter 6.2.8
    scenario = Single_Wall_Thick()  # chapter 5.1.7
    scenario = Single_Wall_Slim()  # chapter 5.1.7

    # for more scenarios look at the scenarios file
    # select a scenario here
    scenario = IFCScenario("../../models/scenarios/Scenario2/scenario2.ifc")

    # load the selected scenario
    scenario.load()

    if export_base_stl:
        WallDetailer.convert_to_stl([], path=output_dir + "base.stl", additional_shapes=[w.get_shape() for w in scenario.walls])

    if export_openings_stl:
        WallDetailer.convert_to_stl([], path=output_dir + "openings.stl", additional_shapes=[o.get_shape() for w in scenario.walls for o in w.openings])

    wall_detailer = WallDetailer(scenario.walls)
    bb = wall_detailer.detail()

    if export_solution_stl:
        WallDetailer.convert_to_stl(bb, path=output_dir + "output.stl", additional_shapes=[])

    if calculate_neighbors:
        brick.calculate_neighborhood(bb)

    if export_solution_json:
        BrickExporter(bb).export_to_json(output_dir + "output.json")

    if use_ontology:
        print("upload into ontology")
        bto = BrickToOntology(bb)

        if deduct_building_plan:
            print("Now deducting building plan")
            result = bto.deduct_building_plan()
            print("Result:", len(result), "brick can be placed with given rulesets. Order:", [b.id for b in result])
            if export_partly_built_stls is not None:
                for i in export_partly_built_stls:
                    building_plan_to_stl(result, i)
