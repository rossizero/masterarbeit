from abc import ABC, abstractmethod
from typing import Tuple, List
import math
import numpy as np
import quaternion

from masonry.brick import BrickInformation

from masonry.Corner import Line


class MaskedArray:
    """
    Represents and 3D Array that can be tweaked via a multiplication-mask and an offset
    """
    def __init__(self, value: np.array = np.array([0, 0, 0]), offset: np.array = np.array([0, 0, 0]),
                 mask: np.array = np.array([1, 1, 1])):
        self.value = value
        self.mask = mask
        self.offset = offset

    def val(self, multiplier: np.array = np.array([0, 0, 0])) -> np.array:
        """
        :param multiplier:
        :return: Calculates value by applying given multiplier to the mask which then is multiplied to the value and moved by /added to given offset

        Example: value = [1, 2, 3], offset = [1, 2, 3], mask = [1, 1, 1]
        val(multiplier=[2, 2, 3]) returns
            [1, 2, 3] + [1, 2, 3] * ([1, 1, 1] * [2, 2, 3]) =
            [1, 2, 3] + [1, 2, 3] * [2, 2, 3] =
            [1, 2, 3] + [2, 4, 9] =
            [3, 6, 12]

        """
        return self.offset + self.value * (self.mask * multiplier)

    def copy(self) -> 'MaskedArray':
        """

        :return: copy of this object
        """
        return MaskedArray(self.value.copy(), self.offset.copy(), self.mask.copy())


class Transformation:
    """
    Just stores two MaskedArrays that represent a translating and a rotating operation
    """
    def __init__(self, translation: MaskedArray, rotation: MaskedArray = MaskedArray(value=np.array([0, 0, 0]))):
        self.rotation = rotation
        self.translation = translation
        self.mask_multiplier = np.array([0, 0, 0])
        self.module = None

    def set_mask_multiplier(self, x: int, y: int, z: int):
        """
        :param x: by what value the mask of the first element of the translation or rotation should be multiplied
        :param y: by what value the mask of the second element of the translation or rotation should be multiplied
        :param z: by what value the mask of the third element of the translation or rotation should be multiplied
        :return: None
        """
        self.mask_multiplier = np.array([x, y, z])

    def get_position(self):
        """
        :return: the position after the mask_multiplier has been applied to self.translation
        """
        return self.translation.val(self.mask_multiplier)

    def get_rotation(self, as_quaternion: bool = True):
        """
        :param as_quaternion: whether you want a quaternion or an euler rotation representation
        :return: the rotation after the mask_multiplier has been applied to self.rotation
        """
        if not as_quaternion:
            return self.rotation.val(self.mask_multiplier)
        return quaternion.from_euler_angles(*self.rotation.val(self.mask_multiplier))

    def copy(self) -> 'Transformation':
        """

        :return: copy of this object
        """
        ret = Transformation(self.translation.copy(), self.rotation.copy())
        ret.set_mask_multiplier(*self.mask_multiplier)
        return ret


class Bond(ABC):
    """
    Superclass of all masonry bonds. It stores the BrickInformation called module,
    which is used when this bond is applied to sth.
    """
    def __init__(self, module: BrickInformation):
        self.module = module

        self.l = module.length
        self.w = module.width
        self.h = module.height
        self.__reset()

    def __reset(self):
        """
        resets counters
        :return:
        """
        self.layer = -1
        self.step = -1
        self.plan = self._get_plan()

        self.repeat_layer = len(self.plan)
        self.repeat_step = len(self.plan[self.layer % self.repeat_layer])

    def __get(self, position: Tuple[int, int] = None) -> Transformation:
        """
        :param position: optional [layer, step in layer] else the internal position is being used
        :return: the Transformation of the plan for this bond at [layer, step]
        """
        layer = max(0, self.layer)
        step = max(0, self.step)
        if position is not None:
            layer, step = position
        ret = self.plan[layer % self.repeat_layer][step % self.repeat_step].copy()
        multiplier = math.floor(step / self.repeat_step)
        ret.set_mask_multiplier(multiplier, multiplier, layer)
        return ret

    def __next(self) -> Transformation:
        """
        move a step further on current layer
        :return:
        """
        self.step += 1
        return self.__get()

    def __up(self, n: int = 1):
        """
        move one layer up (resets the current step)
        :return:
        """
        self.layer += n
        self.repeat_step = len(self.plan[self.layer % self.repeat_layer])
        self.step = -1

    @abstractmethod
    def _get_plan(self) -> List[List[Transformation]]:
        """
        :return: the plan for this masonry bond
        first list holds information for each layer
        each layer holds information about each brick being used by this layer
        You only need to add neccessary information to this layout plan since it is being looped over by [layer, step]
        You will find examples below
        """
        pass

    @abstractmethod
    def _get_corner_plan(self) -> List[List[Transformation]]:
        """
        :return: the plan for a corner using this masonry bond
        """
        pass

    @abstractmethod
    def _get_ending_plan(self) -> List[List[Transformation]]:
        """
        :return: the plan for a corner using this masonry bond
        """
        pass

    def apply_corner(self, layer: int = 0) -> List[Transformation]:
        height = self.module.height
        plan = self._get_corner_plan()

        ret = []
        for t in plan[layer % len(plan)].copy():
            tf = t.copy()
            tf.set_mask_multiplier(0, 0, 1)
            ret.append(tf)

        return ret

    def apply(self, length, width, height, fill_left: bool = False, fill_right: bool = False, layer: int = 0, x_offset: float = 0.0) -> List[Transformation]:
        """
        Fills given dimensions with set layout plan
        :param length: length of wall we want to be filled with this masonry bond
        :param width: width of wall we want to be filled with this masonry bond
        :param height: height of wall we want to be filled with this masonry bond
        :return: a list of Transformations for each brick
        """
        num_layers = int(height / self.h)
        leftover_layer = height % self.h

        self.__reset()
        self.__up(layer)

        ret = []
        for j in range(num_layers):
            self.__up()
            num_bricks, leftover_left, leftover_right = self.bricks_in_layer(self.layer, length + x_offset)
            leftover_left = length

            for i in range(num_bricks):
                tf = self.__next()
                tf.module = self.module
                tf.translation.offset[0] -= x_offset
                if tf.get_position()[0] >= 0:
                    ret.append(tf)
                    leftover_left = min(leftover_left, tf.get_position()[0])

            leftover_right = round(leftover_right, 6)  # necessary because sometimes too small for the occ backend to handle
            leftover_left = round(leftover_left, 6)
            if leftover_left > 0.0 and fill_left:
                tf = Transformation(MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([1, 0, 1])))
                tf.module = BrickInformation(leftover_left, width, self.module.height)
                if leftover_left < width:
                    tf.rotation = MaskedArray(offset=np.array([0, 0, math.pi / 2]))
                tf.set_mask_multiplier(1, 1, self.layer)
                ret.append(tf)
            if leftover_right > 0.0 and fill_right:
                tf = Transformation(MaskedArray(value=np.array([length-leftover_right, 0, self.h]), mask=np.array([1, 0, 1])))
                if leftover_right < width:
                    tf.rotation = MaskedArray(offset=np.array([0, 0, math.pi / 2]))
                tf.module = BrickInformation(leftover_right, width, self.module.height)
                tf.set_mask_multiplier(1, 1, self.layer)
                ret.append(tf)
        return ret

    def bricks_in_layer(self, layer: int, length: float) -> int:
        """
        :param layer: num of layer (0 is at floor)
        :param length: length of the wall
        :param x_offset: offset in x direction
        :return: number of bricks that fit into given length of the wall by following layout plan for given layer
        """
        transformations = self.plan[layer % self.repeat_layer].copy()
        #transformations.sort(key=lambda x: self.module.get_rotated_dimensions(x.get_rotation())[0])
        counter = 0
        tf = transformations[counter].copy()
        multiplier = 0
        tf.set_mask_multiplier(multiplier, multiplier, layer)
        pos = tf.get_position()
        l = self.module.get_rotated_dimensions(tf.get_rotation())[0]

        leftover_left = pos[0]
        leftover_right = 0  #pos[0] + l

        while pos[0] + l <= length:  # TODO calculate instead of expensive "exploration"
            counter += 1
            leftover_right = max(leftover_right, pos[0] + l)
            leftover_left = min(leftover_left, pos[0])

            tf = transformations[counter % len(transformations)].copy()
            multiplier = math.floor(counter / float(len(transformations)))
            tf.set_mask_multiplier(multiplier, multiplier, layer)
            pos = tf.get_position()
            l = self.module.get_rotated_dimensions(tf.get_rotation())[0]
        return counter, leftover_left, length - leftover_right


class BlockBond(Bond):
    def __init__(self, module: BrickInformation):
        super(BlockBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(translation=MaskedArray(value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1]))),
                Transformation(
                    translation=MaskedArray(offset=np.array([0, self.w, 0]), value=np.array([self.l, 0, self.h]),
                                            mask=np.array([1, 0, 1]))),
            ],
            [
                Transformation(
                    translation=MaskedArray(offset=np.array([self.w * 0.5, 0, 0]), value=np.array([self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ]
        ]
        return plan


class StrechedBond(Bond):
    def __init__(self, module: BrickInformation, offset: float = 0.5, schleppend: bool = True):
        self.offset = offset
        self.schleppend = schleppend
        super(StrechedBond, self).__init__(module)

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
                                                       value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1]))),
            ])
            plan.append([
                Transformation(translation=MaskedArray(offset=np.array([(self.l * self.offset), 0, 0]),
                                                       value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1])))
            ])
        return plan

    def _get_corner_plan(self) -> List[List[Transformation]]:
        plan = []
        plan.append([
            Transformation(translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])))
        ])
        plan.append([
            Transformation(
                translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])),
                rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
        ])

        return plan

    def _get_ending_plan(self) -> List[List[Transformation]]:
        plan = []
        plan.append([
            Transformation(translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])))
        ])
        plan.append([
            Transformation(
                translation=MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([0, 0, 1])),
                rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
        ])

        return plan


class HeadBond(Bond):
    def __init__(self, module: BrickInformation):
        super(HeadBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(
                    translation=MaskedArray(offset=np.array([0, 0, 0]), value=np.array([self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ],
            [
                Transformation(
                    translation=MaskedArray(offset=np.array([self.w * 0.5, 0, 0]), value=np.array([self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ]
        ]
        return plan

    def _get_corner_plan(self) -> List[List[Transformation]]:
        plan = []
        return plan

    def _get_ending_plan(self) -> List[List[Transformation]]:
        plan = []
        return plan


class GothicBond(Bond):
    def __init__(self, module: BrickInformation):
        super(GothicBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(
                    translation=MaskedArray(value=np.array([self.l + self.w, 0, self.h]), mask=np.array([1, 0, 1]))),
                Transformation(
                    translation=MaskedArray(value=np.array([self.l + self.w, 0, self.h]), mask=np.array([1, 0, 1]),
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
        return plan

    def _get_ending_plan(self) -> List[List[Transformation]]:
        plan = []
        return plan


class CrossBond(Bond):
    def __init__(self, module: BrickInformation):
        super(CrossBond, self).__init__(module)

    def _get_plan(self) -> List[List[Transformation]]:
        plan = [
            [
                Transformation(translation=MaskedArray(value=np.array([self.l, 0, self.h]), mask=np.array([1, 0, 1]))),
                Transformation(
                    translation=MaskedArray(offset=np.array([0, self.w, 0]), value=np.array([self.l, 0, self.h]),
                                            mask=np.array([1, 0, 1]))),
            ],
            [
                Transformation(
                    translation=MaskedArray(offset=np.array([self.w * 0.5, 0, 0]), value=np.array([self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ],
            [
                Transformation(
                    translation=MaskedArray(offset=np.array([self.l * 0.5, 0, 0]), value=np.array([self.l, 0, self.h]),
                                            mask=np.array([1, 0, 1]))),
                Transformation(
                    translation=MaskedArray(offset=np.array([self.l * 0.5, self.w, 0]),
                                            value=np.array([self.l, 0, self.h]),
                                            mask=np.array([1, 0, 1]))),
            ],
            [
                Transformation(
                    translation=MaskedArray(offset=np.array([self.w * 0.5, 0, 0]), value=np.array([self.w, 0, self.h]),
                                            mask=np.array([1, 0, 1])),
                    rotation=MaskedArray(offset=np.array([0, 0, math.pi / 2]))),
            ],
        ]
        return plan

    def _get_corner_plan(self) -> List[List[Transformation]]:
        plan = []
        return plan

    def _get_ending_plan(self) -> List[List[Transformation]]:
        plan = []
        return plan