from masonry.bond.abstract_bond import Bond, Transformation, MaskedArray
from masonry.brick import BrickInformation
from typing import List

import numpy as np
import math


class StretchedBond(Bond):
    def __init__(self, module: BrickInformation, offset: float = 0.5, schleppend: bool = False):
        self.offset = offset
        self.schleppend = schleppend
        super(StretchedBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = []
        if self.schleppend:
            n = int(1.0 / self.offset)
            for i in range(n):
                plan.append([
                    Transformation(translation=MaskedArray(offset=np.array([(self.l / n) * i, 0, 0]),
                                                           value=np.array([self.l, 0, self.h]),
                                                           mask=np.array([1, 0, 1])))
                ])
        else:
            plan.append([
                Transformation(translation=MaskedArray(offset=np.array([0, 0, 0]),
                                                       value=np.array([self.l, 0, self.h]),
                                                       mask=np.array([1, 0, 1]))),
            ])
            plan.append([
                Transformation(translation=MaskedArray(offset=np.array([(self.l * self.offset), 0, 0]),
                                                       value=np.array([self.l, 0, self.h]),
                                                       mask=np.array([1, 0, 1])))
            ])
        return plan

    def _get_corner_plan(self) -> List[List[Transformation]]:
        plan = []
        if self.schleppend:
            if self.offset == 0.5:
                plan.append([
                    Transformation(
                        translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])),
                        rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
                ])

                plan.append([
                    Transformation(translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])))
                ])
        else:
            if self.offset == 0.5:
                plan.append([
                    Transformation(
                        translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])),
                        rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
                ])

                plan.append([
                    Transformation(translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])))
                ])
            elif self.offset == 0.25:
                plan.append([
                    Transformation(
                        translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])),
                        rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2])))
                ])

                plan.append([
                    Transformation(translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])))
                ])
                for tfs in plan:
                    for tf in tfs:
                        tf.module = BrickInformation(self.module.length * 3 / 4.0, self.module.width,
                                                     self.module.height, grid=np.array([1, 1, 0.5]))

        return plan
