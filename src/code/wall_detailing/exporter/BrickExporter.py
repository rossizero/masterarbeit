from typing import List
import json

import numpy as np
import quaternion

from wall_detailing.masonry.brick import Brick, Neighbor


class BrickExportInformation:
    def __init__(self, brick: Brick):
        self.shape = [brick.length, brick.width, brick.height]
        self.position = np.round(brick.center(), decimals=6).tolist()
        self.rotation = quaternion.as_float_array(brick.orientation).tolist()
        self.neighbors = {}
        self.depends_on = []

        self.id = brick.id

        for key in brick.neighbors:
            self.neighbors[key] = []
            for b in brick.neighbors[key]:
                self.neighbors[key].append(b.id)

        for b in brick.neighbors[Neighbor.BOTTOM]:
            self.depends_on.append(b.id)


class BrickExporter:
    def __init__(self, bricks: List[Brick]):
        self.bricks = bricks

        # set ids
        idd = 1
        for b in bricks:
            b.id = idd
            idd += 1

    def export_to_json(self, path: str):
        """
        Exports the bricks to a json file
        """
        ret = {}
        for b in self.bricks:
            ret[b.id] = BrickExportInformation(b)

        with open(path, 'w') as outfile:
            json.dump(ret, outfile, default=lambda o: o.__dict__, sort_keys=True, indent=4)
