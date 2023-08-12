from abc import ABC, abstractmethod
from typing import Tuple, List

import numpy as np

from masonry_bonds.brick import BrickInformation


class MaskedArray:
    def __init__(self, value: np.array, offset: np.array = np.array([0, 0, 0]), mask: np.array = np.array([0, 0, 0])):
        self.value = value
        self.mask = mask
        self.offset = offset

    def val(self, multiplier: np.array = np.array([0, 0, 0])) -> np.array:
        return self.offset + self.value * (self.mask * multiplier)


class Transformation:
    def __init__(self, translation: MaskedArray, rotation: MaskedArray = MaskedArray(value=np.array([0, 0, 0]))):
        self.rotation = rotation
        self.translation = translation
        self.mask = np.array([0, 0, 0])

    def set_mask_multiplier(self, x: int, y: int, z: int):
        self.mask = [x, y, z]

    def get_position(self):
        return self.translation.val(self.mask)

    def get_rotation(self):
        return self.rotation.val(self.mask)


class Bond(ABC):
    def __init__(self, module: BrickInformation):
        self.module = module

        self.l = module.length
        self.w = module.width
        self.h = module.height

        self.layer = -1
        self.step = -1
        self.plan = self._get_plan()
        self.repeat_layer = len(self.plan)
        self.repeat_step = len(self.plan[self.layer % self.repeat_layer])

    def get(self, position: Tuple[int, int] = None):
        layer = max(0, self.layer)
        step = max(0, self.step)
        if position is not None:
            layer, step = position
        ret = self.plan[layer % self.repeat_layer][step % self.repeat_step]
        ret.set_mask_multiplier(step, step, layer)
        print("layer", layer)
        return ret

    def next(self):
        self.step += 1
        return self.get()

    def up(self):
        self.layer += 1
        self.repeat_step = len(self.plan[self.layer % self.repeat_layer])
        self.step = -1
        return self.get()

    @abstractmethod
    def _get_plan(self) -> List[List[Transformation]]:
        pass


class BlockBond(Bond):
    def __init__(self, module: BrickInformation):
        super(BlockBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(translation=MaskedArray(value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1]))),
                Transformation(translation=MaskedArray(offset=np.array([0, self.w, self.h]), value=np.array([self.l, 0, 0]), mask=np.array([1, 0, 1]))),
            ],
            [
                Transformation(translation=MaskedArray(offset=np.array([self.w * 0.5, 0, self.h]), value=np.array([self.w, 0, 0]), mask=np.array([1, 0, 1])),
                               rotation=MaskedArray(np.array([0, 0, 90]))),
            ]
        ]
        return plan


class StrechedBond(Bond):
    def __init__(self, module: BrickInformation):
        super(StrechedBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(translation=MaskedArray(value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1])))
            ],
            [
                Transformation(translation=MaskedArray(offset=np.array([self.l / 2, 0, 0]), value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1])))
            ]
        ]
        return plan


class CrossBond(Bond):
    def __init__(self, module: BrickInformation):
        super(CrossBond, self).__init__(module)

    def __get_plan(self) -> List[List[Transformation]]:
        return []
