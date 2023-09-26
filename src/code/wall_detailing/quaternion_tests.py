import math

import numpy as np
import quaternion


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0.w, quaternion0.x, quaternion0.y, quaternion0.z
    w1, x1, y1, z1 = quaternion1.w, quaternion1.x, quaternion1.y, quaternion1.z

    return np.quaternion(-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0)


if __name__ == "__main__":
    q1 = quaternion.z.copy()
    angle = math.radians(90.0)
    v1 = np.array([1.0, 0.0, 0.0])
    v3 = np.array([0.0, 0.0, 1.0])

    w1 = quaternion.from_euler_angles(0.0, 0.0, 0.0)
    w1_ = quaternion.from_euler_angles(0.0, 0.0, 0.0)
    w2 = quaternion.from_euler_angles(angle, 0.0, 0.0)
    w3 = quaternion.from_euler_angles(0.0, angle, 0.0)
    w4 = quaternion.from_euler_angles(0.0, 0.0, angle)

    print(w1, w2, w3)
    print(w1.w * w2.w + w1.x * w2.x + w1.y * w2.y + w1.z * w2.z, angle)
    rot1 = quaternion.rotate_vectors(w1, v1.copy())
    rot1_ = quaternion.rotate_vectors(w1_, v1.copy())
    rot2 = quaternion.rotate_vectors(w2, v1.copy())
    rot3 = quaternion.rotate_vectors(w3, v1.copy())
    rot4 = quaternion.rotate_vectors(w4, v1.copy())

    rot1_z = quaternion.rotate_vectors(w1, v3.copy())
    rot1__z = quaternion.rotate_vectors(w1_, v3.copy())
    rot2_z = quaternion.rotate_vectors(w2, v3.copy())
    rot3_z = quaternion.rotate_vectors(w3, v3.copy())
    rot4_z = quaternion.rotate_vectors(w4, v3.copy())

    print(rot1, rot2)

    print("test", np.dot(rot1_z, rot1__z))

    print(np.dot(rot1, rot2))
    print(np.dot(rot1_z, rot2_z))

    print(np.dot(rot1, rot3))
    print(np.dot(rot1_z, rot3_z))

    print(np.dot(rot1, rot4))
    print(np.dot(rot1_z, rot4_z))

    rotation_matrix1 = quaternion.as_rotation_matrix(w1)
    rotation_matrix2 = quaternion.as_rotation_matrix(w2)
    rotation_matrix3 = quaternion.as_rotation_matrix(w3)
    rotation_matrix4 = quaternion.as_rotation_matrix(w4)

    x_axis = np.array([1, 0, 0])
    rotated_x_axis1 = np.dot(rotation_matrix1, x_axis)
    rotated_x_axis2 = np.dot(rotation_matrix2, x_axis)
    rotated_x_axis3 = np.dot(rotation_matrix3, x_axis)
    rotated_x_axis4 = np.dot(rotation_matrix4, x_axis)

    print(np.dot(rotated_x_axis1, rotated_x_axis2))
    print(np.dot(rotated_x_axis1, rotated_x_axis3))
    print(np.dot(rotated_x_axis1, rotated_x_axis4))

    print("----")
    q90 = quaternion.from_euler_angles(0, 0, math.pi / 2.0)
    arr = np.array([1, 0, 0])
    print(np.dot(quaternion.as_rotation_matrix(q90), arr))
    print(np.dot(quaternion.as_rotation_matrix(q90), arr.T).T)
    print(q90)
    print(quaternion_multiply(q90, q90))
    print(quaternion_multiply(quaternion.from_euler_angles(0, 0, 0), q90))

    print("---------")
    q90 = quaternion.from_euler_angles(0, 0, math.pi / 2.0)
    print(quaternion.as_euler_angles(q90))
    q180 = q90 * q90
    print(quaternion.as_euler_angles(q180))
    q180 /= q90
    print(quaternion.as_euler_angles(q180))
    vec = [2, 1, 0]
    print(quaternion.rotate_vectors(q90, vec))
    print(quaternion.rotate_vectors(quaternion.from_euler_angles(0, 0, -math.pi / 2.0), vec))
    print(quaternion.rotate_vectors(q90*q90, vec))
    print(quaternion.rotate_vectors(q90*quaternion.from_euler_angles(0, 0, -math.pi / 2.0), vec))
    print(q90, q90.inverse(), q90.inverse() * q90, q90 * q90.inverse())
