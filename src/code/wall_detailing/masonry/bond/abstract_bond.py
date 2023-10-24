from abc import ABC, abstractmethod
from typing import Tuple, List, Union, Any
from masonry.brick import BrickInformation

import math
import numpy as np
import quaternion


class MaskedArray:
    """
    Represents and 3D Array that can be tweaked via a multiplication-mask and an offset
    """
    def __init__(self, value: np.array = np.array([0.0, 0.0, 0.0]), offset: np.array = np.array([0.0, 0.0, 0.0]),
                 mask: np.array = np.array([1, 1, 1])):
        self.value = value.astype(dtype=float)
        self.mask = mask
        self.offset = offset.astype(dtype=float)

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
        self.layer = 0
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

    def get_corner_plan_repeat_step(self):
        return len(self._get_corner_plan())

    def apply_corner(self, layer: int = 0) -> List[Transformation]:
        """
        :param layer: index for the layer of the corner plan we need
        :return: a list of Transformations for each brick
        """
        plan = self._get_corner_plan()

        ret = []
        for t in plan[layer % len(plan)].copy():
            tf = t.copy()
            tf.set_mask_multiplier(0, 0, layer)
            tf.module = t.module if t.module is not None else self.module
            ret.append(tf)

        return ret

    def get_corner_length(self, layer: int = 0, rotation: np.quaternion = np.quaternion(1, 0, 0, 0)) -> float:
        """
        :param layer: index for the layer of the corner plan we need
        :param rotation: how should the corner plan be rotated
        :return: the maximal x coordinate of the corner plan
        """
        plan = self._get_corner_plan()
        ret = 0
        lll = []
        for t in plan[layer % len(plan)].copy():
            module = t.module if t.module is not None else self.module
            l = module.get_rotated_dimensions(t.get_rotation())
            a = l
            l = quaternion.rotate_vectors(rotation, l)
            b = l
            #l = abs(l[0])
            l = abs(l[0])
            c = l
            t.set_mask_multiplier(1, 0, 1)
            pos = quaternion.rotate_vectors(rotation, t.get_position())
            d = pos
            l += pos[0]
            e = l
            ret = max(ret, l)
            lll.append(l)
        #print(lll)
        return round(ret, 6)

    def leftover_of_layer(self, length: float, layer: int = 0, x_offset: float = 0.0, reversed: bool = False):
        self.__reset()
        self.__up(layer)

        bricks, leftover_left, leftover_right = self.bricks_in_layer(layer, length, x_offset, reversed)

        return leftover_left, leftover_right, len(bricks)

    def apply_layer(self, length, width, fill_left: bool = False, fill_right: bool = False, layer: int = 0, x_offset: float = 0.0, reversed: bool = False) -> List[Transformation]:
        """
        Fills given dimensions with set layout plan
        :param length: length of wall we want to be filled with this masonry bond
        :param width: width of wall we want to be filled with this masonry bond
        :param fill_left: if we wish to fill holes on the left with custom brick sizes
        :param fill_right: if we wish to fill holes on the right with custom brick sizes
        :param layer: the index of the plan we want to use
        :param x_offset: if the layer's left edge is not at x = 0
        :param reversed: if the bricks are supposed to be placed from right to left
        :return: a list of Transformations for each brick
        """
        bricks, leftover_left, leftover_right = self.bricks_in_layer(layer, length, x_offset, reversed)

        if leftover_left + leftover_right == length:
            # if we only want to fill one side, but the layer only consists of leftovers
            # -> fill the whole layer from one side
            if fill_left and not fill_right:
                leftover_left += leftover_right
                leftover_right = 0
            elif fill_right and not fill_left:
                leftover_right += leftover_left
                leftover_left = 0

        if leftover_left > 0.0 and fill_left:
            tf = Transformation(MaskedArray(value=np.array([0, 0, self.h]), mask=np.array([1, 0, 1])))
            tf.module = BrickInformation(leftover_left, width, self.module.height)
            if leftover_left < width:
                tf.rotation = MaskedArray(offset=np.array([0, 0, math.pi / 2]))
            tf.set_mask_multiplier(1, 1, layer)
            bricks.append(tf)

        if leftover_right > 0.0 and fill_right:
            tf = Transformation(MaskedArray(value=np.array([length-leftover_right, 0, self.h]), mask=np.array([1, 0, 1])))
            tf.module = BrickInformation(leftover_right, width, self.module.height)
            if leftover_right < width:
                tf.rotation = MaskedArray(offset=np.array([0, 0, math.pi / 2]))
            tf.set_mask_multiplier(1, 1, layer)
            bricks.append(tf)
        return bricks

    def num_bricks_in_length(self, layer: int, length: float):
        """
        :param layer: the plan layer we look at
        :param length: the length we want to apply the bond to
        :return: number of bricks in length, leftover on both sides
        """
        # get plan and starting position
        transformations = self.plan[layer % self.repeat_layer].copy()
        tf = transformations[0].copy()
        tf.set_mask_multiplier(0, 0, layer)
        pos = tf.get_position()

        leftover_left = pos[0]  # leftover_left usually stays at this value
        leftover_right = leftover_left  # leftover_right is the "x-coordinate" of the last bricks right edge
        brick_length = self.module.get_rotated_dimensions(tf.get_rotation())[0]

        counter = 0
        multiplier = 0

        # iterate over the layerplan while increasing the multiplier
        # until we find a brick that reaches over the given length
        while pos[0] + brick_length <= length:
            counter += 1
            leftover_right = max(leftover_right, pos[0] + brick_length)
            leftover_left = min(leftover_left, pos[0])

            tf = transformations[counter % len(transformations)].copy()
            multiplier = math.floor(counter / float(len(transformations)))
            tf.set_mask_multiplier(multiplier, multiplier, layer)
            pos = tf.get_position()

            brick_length = self.module.get_rotated_dimensions(tf.get_rotation())[0]
        return counter, round(leftover_left, 6), round(leftover_right, 6)

    def bricks_in_layer(self, layer: int, length: float, x_offset: float = 0.0, reversed: bool = False) -> Tuple[List[Transformation], float, float]:
        """
        :param layer: index of layer plan (0 is at floor)
        :param length: length of the wall
        :param x_offset: offset in x direction
        :param reversed: if the bricks are supposed to be placed from right to left
        :return: number of bricks that fit into given length of the wall by following layout plan for given layer
        """
        # check how many bricks are in this layers length + it's x_offset
        # aka we act like the layer is longer as it might be, to be able to cut out bricks correctly later
        counter, leftover_left, leftover_right = self.num_bricks_in_length(layer, length + x_offset)
        ret = []

        # if there are no whole bricks at all
        if counter == 0:
            leftover_right = length - leftover_left
        # if there are some whole bricks in the length + x_offset
        else:
            self.__reset()
            self.__up(layer)

            # now let's check if there are any bricks that have x positions greater than given x_offset
            any_bricks = False
            for i in range(counter):
                tf = self.__next()
                tf.module = self.module

                if tf.get_position()[0] >= x_offset:
                    diff = tf.get_position()[0] - x_offset
                    if not any_bricks:
                        any_bricks = True
                        leftover_left = diff

                    leftover_left = min(leftover_left, diff)
                    tf.translation.offset[0] -= x_offset
                    ret.append(tf)
            # in case there are any bricks
            if any_bricks:
                # rounding necessary because sometimes too small for the occ backend to handle
                leftover_right = round(leftover_right - x_offset, 6)
                leftover_right = length - leftover_right
                leftover_left = round(leftover_left, 6)
            # otherwise do a slightly different calculation
            else:
                leftover_left = length - leftover_left
                leftover_right = length - leftover_left

        # reverse tfs to build from left to right
        if reversed:
            leftover_right, leftover_left = leftover_left, leftover_right

            for tf in ret:
                brick_length = self.module.get_rotated_dimensions(tf.get_rotation())[0]
                tf.translation.offset[0] = length - tf.translation.offset[0] - brick_length
                tf.mask_multiplier[0] *= -1

        # in case we want to build the layer from right to left
        return ret, leftover_left, leftover_right
