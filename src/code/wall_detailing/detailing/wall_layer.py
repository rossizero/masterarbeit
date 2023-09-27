import numpy as np
import quaternion

from typing import List


class WallLayer:
    """
    A straight layer of a wall
    """
    def __init__(self, parent: 'WallLayerGroup', length: float, translation: np.array = np.array([0.0, 0.0, 0.0]), height: float = None):
        self.parent = parent
        self.translation = translation  # relative to walls translation
        self.height = parent.module.height if height is None else height
        self.length = length

        self.left_connections: List['WallLayer'] = []
        self.right_connections: List['WallLayer'] = []

    def combine(self, other: 'WallLayer'):
        """
        Combines two wall layers to one. Only makes sense if the two layers have the same orientation,
        module and are on the same height
        :param other: the other wall layer that will be included into self
        :return: nothing
        """
        total_length = self.length + other.length
        global_mid = (self.center * self.length + other.center * other.length) / total_length
        local_mid = global_mid - self.parent.get_translation()
        local_mid = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), local_mid)
        self.translation = local_mid
        self.length = total_length

    def get_layer_index(self):
        """
        :return: layer index that self has inside its assigned walllayergroup
        """
        for i, l in enumerate(self.parent.get_sorted_layers()):
            if self in l:
                return i
        return None

    @property
    def left_edge(self):
        """
        :return: the left edge of this layer in world coordinates
        """
        return self.get_left_edge()

    def get_left_edge(self, relative: bool = False) -> np.array:
        """
        :param relative: whether we want the relative or world coordinates of the center
        :return: coordinates of the left edge of this layer
        """
        ret = self.translation.copy()
        ret[0] -= self.length/2.0

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    @property
    def right_edge(self):
        """
        :return: the right edge of this layer in world coordinates
        """
        return self.get_right_edge()

    def get_right_edge(self, relative: bool = False) -> np.array:
        """
        :param relative: whether we want the relative or world coordinates of the center
        :return: coordinates of the right edge of this layer
        """
        ret = self.translation.copy()
        ret[0] += self.length / 2.0

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    @property
    def relative_x_offset(self) -> float:
        """
        :return: the difference between the lowest x coordinate of all layers of the parent and self
        """
        a = min(self.get_left_edge(True)[0], self.get_right_edge(True)[0])
        b = self.parent.get_lowest_local_x()
        return a - b

    @property
    def center(self) -> np.array:
        """
        :return: the center of this layer in world coordinates
        """
        return self.get_center()

    def get_center(self, relative: bool = False) -> np.array:
        """
        :param relative: whether we want the relative or world coordinates of the center
        :return: coordinates of the center of this wall_layer
        """
        ret = self.translation.copy()

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    def is_touching(self, other: 'WallLayer', tolerance: float = 1.e-8) -> bool:
        """
        Checks if self and other edges are (nearly) touching
        :param other: the other wall_layer
        :param tolerance: how close the two edge points have to be
        :return:
        """
        a = np.allclose(self.right_edge, other.left_edge, atol=tolerance)
        b = np.allclose(self.right_edge, other.right_edge, atol=tolerance)
        c = np.allclose(self.left_edge, other.left_edge, atol=tolerance)
        d = np.allclose(self.left_edge, other.right_edge, atol=tolerance)
        return a or b or c or d

    def is_above_or_below(self, other: 'WallLayer', height: float = 0.0) -> bool:
        """
        Checks if self is exactly one module height above or below another layer
        :param other: other wall_layer
        :param height:
        :return:
        """
        a = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), self.left_edge)
        b = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), self.right_edge)

        c = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), other.left_edge)
        d = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), other.right_edge)

        x_between = a[0] <= c[0] <= b[0] or c[0] <= a[0] <= d[0] or a[0] <= d[0] <= b[0] or c[0] <= b[0] <= d[0]
        y_same = np.allclose(a[1], [b[1], c[1], d[1]])

        aa = abs(a[2] - c[2])
        bb = abs(a[2] - d[2])
        cc = abs(b[2] - c[2])
        dd = abs(b[2] - d[2])
        z_diff_same = np.allclose(aa, [bb, cc, dd])
        z_diff_right = np.isclose(aa, height)

        return x_between and y_same and z_diff_same and z_diff_right

    def reduce_length(self, length: float, from_left: bool = False, from_right: bool = False) -> bool:
        """
        reduces the length of this layer by given length.
        If from_left and from_right have the same value, the center stays at its old position, otherwise it will move so
        that either the left or the right edge stays at its old position.
        :param length: new length = self.length - length if incoming length <= length
        :param from_left: if True, center will be moved so that the right corner stays at its prior position
        :param from_right: if True, center will be moved so that the left corner stays at its prior position
        :return: if operation was sucessful
        """
        if length >= self.length:
            return False

        # move center
        if not from_left == from_right:
            if from_left:
                self.translation[0] += length / 2.0
            else:
                self.translation[0] -= length / 2.0
        self.length -= length
        return True

    def move_edge(self, start_point: np.array, length: float = 0):
        """
        Moves either the right or left edge in the direction of the center
        :param start_point: a point we start the movement from (should lie on this layers "line" of direction)
        :param length: how far we want to move the edge
        :return: nothing
        """
        is_left = np.linalg.norm(start_point - self.left_edge) < np.linalg.norm(start_point - self.right_edge)
        local_start_point = start_point - self.parent.get_translation()
        local_start_point = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), local_start_point)

        right = self.get_right_edge(True)
        left = self.get_left_edge(True)

        if is_left:
            x = local_start_point - left
        else:
            x = right - local_start_point

        x = round(x[0], 6)
        length -= (abs(x) - x)
        self.reduce_length(length, from_left=is_left, from_right=not is_left)

    def __lt__(self, other):
        """
        Needed to be able to sort layers by their parents
        :param other:
        :return:
        """
        if type(other) is WallLayer:
            return self.parent < other.parent
        return True
