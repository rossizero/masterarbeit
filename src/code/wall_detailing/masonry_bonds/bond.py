from abc import ABC, abstractmethod
from typing import Tuple, List
import math
import numpy as np
import quaternion

from masonry_bonds.brick import BrickInformation


class MaskedArray:
    def __init__(self, value: np.array = np.array([0, 0, 0]), offset: np.array = np.array([0, 0, 0]), mask: np.array = np.array([1, 1, 1])):
        self.value = value
        self.mask = mask
        self.offset = offset

    def val(self, multiplier: np.array = np.array([0, 0, 0])) -> np.array:
        return self.offset + self.value * (self.mask * multiplier)

    def copy(self) -> 'MaskedArray':
        return MaskedArray(self.value.copy(), self.offset.copy(), self.mask.copy())


class Transformation:
    def __init__(self, translation: MaskedArray, rotation: MaskedArray = MaskedArray(value=np.array([0, 0, 0]))):
        self.rotation = rotation
        self.translation = translation
        self.mask_multiplier = np.array([0, 0, 0])

    def set_mask_multiplier(self, x: int, y: int, z: int):
        self.mask_multiplier = np.array([x, y, z])

    def get_position(self):
        return self.translation.val(self.mask_multiplier)

    def get_rotation(self, as_quaternion: bool = True):
        if not as_quaternion:
            return self.rotation.val(self.mask_multiplier)
        return quaternion.from_euler_angles(*self.rotation.val(self.mask_multiplier))

    def copy(self) -> 'Transformation':
        ret = Transformation(self.translation.copy(), self.rotation.copy())
        ret.set_mask_multiplier(*self.mask_multiplier)
        return ret


class Bond(ABC):
    def __init__(self, module: BrickInformation):
        self.module = module

        self.l = module.length
        self.w = module.width
        self.h = module.height
        self.__reset()

    def __reset(self):
        self.layer = -1
        self.step = -1
        self.plan = self._get_plan()
        self.repeat_layer = len(self.plan)
        self.repeat_step = len(self.plan[self.layer % self.repeat_layer])

    def __get(self, position: Tuple[int, int] = None):
        layer = max(0, self.layer)
        step = max(0, self.step)
        if position is not None:
            layer, step = position
        ret = self.plan[layer % self.repeat_layer][step % self.repeat_step].copy()
        ret.set_mask_multiplier(step, step, layer)
        return ret

    def __next(self):
        self.step += 1
        return self.__get()

    def __up(self):
        self.layer += 1
        self.repeat_step = len(self.plan[self.layer % self.repeat_layer])
        self.step = -1

    @abstractmethod
    def _get_plan(self) -> List[List[Transformation]]:
        pass

    def apply(self, length, width, height) -> List[Transformation]:
        length, width = max(length, width), min(length, width)
        num_layers = int(height / self.h)
        leftover_layer = height % self.h
        # print(num_layers, leftover_layer)
        ret = []
        for j in range(num_layers):
            self.__up()
            num_bricks = self.bricks_in_layer(j, length)
            # print("num bricks in layer", j, ":", num_bricks)
            for i in range(self.bricks_in_layer(j, length)):
                tf = self.__next()
                ret.append(tf)
        return ret

    def bricks_in_layer(self, layer: int, length: float) -> int:
        transformations = self.plan[layer % self.repeat_layer]
        counter = 0
        multiplier = 0
        tf = transformations[counter].copy()
        tf.set_mask_multiplier(multiplier, multiplier, layer)
        pos = tf.get_position()

        while pos[0] + self.l <= length: # TODO calculate instead of expensive "exploration"
            counter += 1
            tf = transformations[counter % len(transformations)].copy()
            multiplier = math.floor(counter / float(len(transformations)))
            tf.set_mask_multiplier(multiplier, multiplier, layer)
            pos = tf.get_position()
        return counter


class BlockBond(Bond):
    def __init__(self, module: BrickInformation):
        super(BlockBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(translation=MaskedArray(value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1]))),
                Transformation(translation=MaskedArray(offset=np.array([-self.l, self.w, 0]), value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1]))),
            ],
            [
                Transformation(translation=MaskedArray(offset=np.array([self.w * 0.5, 0, 0]), value=np.array([self.w, 0, self.h]), mask=np.array([1, 0, 1])),
                               rotation=MaskedArray(offset=np.array([0, 0, math.pi/2]))),
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
