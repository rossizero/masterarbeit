import numpy as np
from OCC.Core.BRep import BRep_Tool
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_VERTEX
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopoDS import TopoDS_Shape, topods


def get_shape_dimensions(shape: TopoDS_Shape, grid: np.array = None):
    """
    :param shape: the shape to get the dimensions from
    :param grid: the grid to snap the dimensions to
    :return: the dimensions of the shape
    """
    edge_explorer = TopExp_Explorer(shape, TopAbs_EDGE)
    vertices = []
    while edge_explorer.More():
        edge = topods.Edge(edge_explorer.Current())
        vertex_explorer = TopExp_Explorer(edge, TopAbs_VERTEX)
        while vertex_explorer.More():
            vertex = topods.Vertex(vertex_explorer.Current())
            vertex_point = BRep_Tool.Pnt(vertex)
            vertices.append(np.array([vertex_point.X(), vertex_point.Y(), vertex_point.Z()]))
            vertex_explorer.Next()
        edge_explorer.Next()

    vertices = np.unique(vertices, axis=0)
    x = max(vertices[:, 0]) - min(vertices[:, 0])
    y = max(vertices[:, 1]) - min(vertices[:, 1])
    z = max(vertices[:, 2]) - min(vertices[:, 2])

    dimensions = np.array([x, y, z])

    # round to nearest grid point
    if grid is not None:
        dimensions = np.array([
            round(x / grid[0]) * grid[0],
            round(y / grid[1]) * grid[1],
            round(z / grid[2]) * grid[2]
        ])
    # sometimes the dimensions are not exactly on the grid, so we round them
    dimensions = np.round(dimensions, decimals=6)

    length = max(dimensions[0], dimensions[1])
    width = min(dimensions[0], dimensions[1])
    height = dimensions[2]
    ret = np.array([length, width, height])
    return ret