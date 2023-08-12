import math
from typing import List, Dict
import numpy as np
import quaternion
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
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


class BrickInformation:
    def __init__(self, length, width, height):
        self.length = max(length, width)
        self.width = min(length, width)
        self.height = height

    def volume(self):
        return self.length * self.width * self.height


class Brick(BrickInformation):
    def __init__(self, length, width, height, position: np.array, rotation: quaternion):
        super().__init__(length, width, height)
        self.position = position
        self.rotation = rotation

    def get_brep_shape(self):
        offset = 0.01
        corner = gp_Pnt(0, 0, 0)

        shape = BRepPrimAPI_MakeBox(corner, self.length - offset, self.width - offset, self.height - offset).Shape()

        # translate to center shape around 0 0 0
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # rotate around local rotation
        transformation = gp_Trsf()
        rotation = quaternion.from_euler_angles(0, 0, math.pi/2.0)
        rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # translate back
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(self.length / 2.0, self.width / 2.0, self.height / 2.0))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # translate to global position
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.position))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # rotate
        transformation = gp_Trsf()
        rotation = gp_Quaternion(self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w)
        transformation.SetRotation(rotation)
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

    def is_cubic(self, shape: TopoDS_Shape):
        # Apply the inverted rotation to our shape to get axis aligned shape
        rotated_box_shape = shape.Reversed()
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(shape.Location().Transformation().TranslationPart().Reversed()))
        rotated_box_shape = BRepBuilderAPI_Transform(rotated_box_shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(shape.Location().Transformation().GetRotation().Inverted())
        rotated_box_shape = BRepBuilderAPI_Transform(rotated_box_shape, transformation).Shape()

        # create a boundingbox around the shape
        bounding_box = Bnd_Box()
        brepbndlib_Add(rotated_box_shape, bounding_box)

        # Get the minimum and maximum coordinates of the bounding box
        xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()
        min_max_array_bbox = np.around(np.array(bounding_box.Get()), decimals=6)

        # Calculate the dimensions of the rotated box
        length = xmax - xmin
        width = ymax - ymin
        height = zmax - zmin

        edge_explorer = TopExp_Explorer(rotated_box_shape, TopAbs_EDGE)
        coords = [[], [], []]
        while edge_explorer.More():
            edge = topods.Edge(edge_explorer.Current())
            vertex_explorer = TopExp_Explorer(edge, TopAbs_VERTEX)
            while vertex_explorer.More():
                vertex = topods.Vertex(vertex_explorer.Current())
                vertex_point = BRep_Tool.Pnt(vertex)
                coords[0].append(vertex_point.X())
                coords[1].append(vertex_point.Y())
                coords[2].append(vertex_point.Z())
                vertex_explorer.Next()
            edge_explorer.Next()

        gprops = GProp_GProps()
        brepgprop_VolumeProperties(shape, gprops)

        # check if boundingbox has the same vertices as our axis aligned shape
        # and if their volumes are equal (if not there are openings inside the shape)
        min_max_array_wall = np.around(np.array([min(coords[0]), min(coords[1]), min(coords[2]),
                                       max(coords[0]), max(coords[1]), max(coords[2])]), decimals=6)
        close = np.allclose(min_max_array_wall, min_max_array_bbox) or np.allclose(min_max_array_bbox, min_max_array_wall)
        return close and np.isclose(gprops.Mass(), length * width * height)

    def detail_wall(self, wall: Wall, bricks: List[BrickInformation]):
        print(wall, bricks)
        brick_ret = []
        if self.is_cubic(wall.occ_shape):
            # Apply the inverted translation and rotation to our shape to get axis aligned shape at [0, 0, 0]
            transformation = gp_Trsf()


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
            tmp = length
            length = max(length, width)
            width = min(tmp, width)
            print(length, width, height)

            # get "biggest" brick TODO maybe there is a better criteria?
            bricks.sort(key=lambda x: x.volume(), reverse=True)
            print(bricks[0].volume())
            module = bricks[0]
            num_layers = int(height / module.height)
            leftover_layer = height % module.height
            print(num_layers, leftover_layer)  # TODO what if leftover
            # whether we use the module's length or width as "Läufer"
            rotate = module.width != width  # TODO
            length_step = module.width if rotate else module.length
            length_steps = int(length / length_step)
            length_leftover = length % length_step
            print(rotate, length_step, length_steps, length_leftover)  # TODO what if leftover

            print(original_translation)
            r = math.pi / 2.0 if not rotate else 0
            for j in range(num_layers):
                offset = j % 2 * length_step / 2.0  # TODO
                for i in range(length_steps - j % 2):
                    pos = np.array([offset + i * length_step, 0, j * module.height]) + original_translation
                    rot = original_rotation
                    print(pos, r, rot)
                    b = Brick(module.length, module.width, module.height, pos, rot)
                    brick_ret.append(b)

            #for b in brick_ret:
            #    b.position += np.array([xmin, ymin, zmin])
            #    b.rotation += original_rotation

        return brick_ret

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
    corner = gp_Pnt(0, 0, 0)

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
    wall = Wall(make_occ_box(10, 1, 5, [1, 1, 1], quaternion.from_euler_angles(1.3, 1.02, math.pi/3)))
    wall.ifc_wall_type = "test"
    brick_information = {"test": [BrickInformation(2, 1, 0.5), BrickInformation(1, 0.5, 0.5)]}
    walls = [wall]
    wall_detailer = WallDetailer(walls, brick_information)
    bb = wall_detailer.detail()
    #b = [Brick(1, 2, 0.5, np.ones(shape=(3,)) * i, quaternion.from_euler_angles(math.pi * 2 / 20.0 * i, 0, 0)) for i in range(80)]
    WallDetailer.convert_to_stl(bb, "output.stl", additional_shapes=[wall.occ_shape])
