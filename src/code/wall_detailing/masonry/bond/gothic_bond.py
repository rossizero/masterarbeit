from masonry.bond.abstract_bond import Bond, Transformation, MaskedArray
from masonry.brick import BrickInformation
from typing import List

import numpy as np
import math



class GothicBond(Bond):
    def __init__(self, module: BrickInformation):
        super(GothicBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(
                    translation=MaskedArray(value=np.array([self.l + self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1]))),
                Transformation(
                    translation=MaskedArray(value=np.array([self.l + self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1]),
                                            offset=np.array([0, self.w, 0]))),
                Transformation(
                    translation=MaskedArray(offset=np.array([self.l, 0, 0]),
                                            value=np.array([self.l + self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ],
            [
                Transformation(translation=MaskedArray(offset=np.array([self.w * 0.5, 0, 0]),
                                                       value=np.array([self.l + self.w, 0, self.h]),
                                                       mask=np.array([1, 0, 1]))),
                Transformation(translation=MaskedArray(offset=np.array([self.w * 0.5, self.w, 0]),
                                                       value=np.array([self.l + + self.w, 0, self.h]),
                                                       mask=np.array([1, 0, 1]))),
                Transformation(
                    translation=MaskedArray(offset=np.array([self.l + self.w * 0.5, 0, 0]),
                                            value=np.array([self.l + self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ]
        ]
        return plan

    def _get_corner_plan(self) -> List[List[Transformation]]:
        plan = []

        plan.append([
            Transformation(
                translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])),
                rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            Transformation(
                translation=MaskedArray(offset=np.array([self.w, 0, 0]), value=np.array([0, 0, self.h]),
                                        mask=np.array([0, 1, 1])),
                rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
        ])

        plan.append([
            Transformation(translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1]))),
            Transformation(translation=MaskedArray(offset=np.array([0, self.w, 0]), value=np.array([0, 0, self.h]),
                                                   mask=np.array([0, 0, 1])))
        ])

        for tfs in plan:
            for tf in tfs:
                tf.module = BrickInformation(self.module.length * 3 / 4.0, self.module.width, self.module.height,
                                             grid=self.module.grid)
        return plan
