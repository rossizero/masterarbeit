import numpy as np


class Tile:
    def __init__(self):
        self.ifc_wall_type = None
        self.faces = []

    def to_mesh(self):
        pass


class CubicTile(Tile):
    def __init__(self):
        super(CubicTile, self).__init__()


class Face:
    def __init__(self, std_vector, angle=np.array([0, 0, 0])):
        self.std_vector = std_vector
        self.angle = angle

        # cutting of Tile
        self.min_vector = []
        self.max_vector = []
        self.step_vector = []


std_cube = CubicTile()
std_cube.faces.append(Face(np.array([0.5, 0, 0])))
std_cube.faces.append(Face(np.array([0, 0.5, 0])))
std_cube.faces.append(Face(np.array([0, 0, 0.5])))
std_cube.faces.append(Face(np.array([-0.5, 0, 0])))
std_cube.faces.append(Face(np.array([0, -0.5, 0])))
std_cube.faces.append(Face(np.array([0, 0, -0.5])))
