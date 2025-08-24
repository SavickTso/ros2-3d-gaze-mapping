import math
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec

ZEDM_FX = 1397.8280029296875
ZEDM_FY = 1397.8280029296875
ZEDM_CX = 958.65673828125
ZEDM_CY = 511.1455


def wait_for_message(msg_type, node: "Node", topic: str, time_to_wait=-1):
    """
    Wait for the next incoming message.

    :param msg_type: message type
    :param node: node to initialize the subscription on
    :param topic: topic name to wait for message
    :param time_to_wait: seconds to wait before returning
    :returns: (True, msg) if a message was successfully received, (False, None) if message
        could not be obtained or shutdown was triggered asynchronously on the context.
    """
    context = node.context
    wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
    wait_set.clear_entities()

    sub = node.create_subscription(msg_type, topic, lambda _: None, 1)
    try:
        wait_set.add_subscription(sub.handle)
        sigint_gc = SignalHandlerGuardCondition(context=context)
        wait_set.add_guard_condition(sigint_gc.handle)

        timeout_nsec = timeout_sec_to_nsec(time_to_wait)
        wait_set.wait(timeout_nsec)

        subs_ready = wait_set.get_ready_entities("subscription")
        guards_ready = wait_set.get_ready_entities("guard_condition")

        if guards_ready:
            if sigint_gc.handle.pointer in guards_ready:
                return False, None

        if subs_ready:
            if sub.handle.pointer in subs_ready:
                msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
                if msg_info is not None:
                    return True, msg_info[0]
    finally:
        node.destroy_subscription(sub)

    return False, None


def get_z_in_matrix(x, y, matrix):
    return matrix[int(matrix.shape[0] / 2) + x][int(matrix.shape[1] / 2) + y]


def _make_homogeneous_rep_matrix(R, t):
    P = np.zeros((4, 4))
    P[:3, :3] = R
    P[:3, 3] = t.reshape(3)
    P[3, 3] = 1
    return P


def read_camera_parameters(camera_id, pkg_path):
    inf = open(
        pkg_path
        + "/config/camera_parameters/camera"
        + str(camera_id)
        + "_intrinsics.dat",
        "r",
    )

    cmtx = []
    dist = []

    line = inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        cmtx.append(line)

    line = inf.readline()
    line = inf.readline().split()
    line = [float(en) for en in line]
    dist.append(line)

    return np.array(cmtx), np.array(dist)


def read_rotation_translation(
    camera_id,
    pkg_path,
    savefolder="/config/camera_parameters/camera",
):
    inf = open(pkg_path + savefolder + str(camera_id) + "_rot_trans" + ".dat", "r")

    inf.readline()
    rot = []
    trans = []
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        rot.append(line)

    inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        trans.append(line)

    inf.close()
    return np.array(rot), np.array(trans)


def get_projection_matrix(camera_id, pkg_path):
    # read camera parameters
    cmtx, dist = read_camera_parameters(camera_id, pkg_path)
    rvec, tvec = read_rotation_translation(camera_id, pkg_path)

    # calculate projection matrix
    P = cmtx @ _make_homogeneous_rep_matrix(rvec, tvec)[:3, :]
    return P


def world_to_cam(Pwx, Pwy, Pwz, Pcam):
    Pw = np.array([Pwx, Pwy, Pwz, 1])
    camcoor = Pcam @ Pw
    return camcoor


def get_projected_cam(x, y, z, p):
    """
    Get the corresponding point in camera coordinate of the point in world coordinate
    """

    pnt1w = [x, y, z]
    # get corresponding point in camera coordinate
    camcoor = world_to_cam(pnt1w[0], pnt1w[1], pnt1w[2], p)
    return camcoor[0] / camcoor[2], camcoor[1] / camcoor[2]


def newton3d(Point1, Point2):
    """
    Compute the x,y coordinate of the point3 based on the derivative of point1 and point2
    """
    d_x = (Point1[0] - Point2[0]) / (Point1[2] - Point2[2])
    d_y = (Point1[1] - Point2[1]) / (Point1[2] - Point2[2])

    z3x = Point2[2] - Point2[0] / d_x
    z3y = Point2[2] - Point2[1] / d_y

    return z3x


def newton3d_depth(Point1, Point2):
    """
    Point2: newer point, in format of (z_virtual, z_err)
    Compute the x coordinate of the zero-point based on the derivative of point1 and point2
    """
    d_err = (Point1[0] - Point2[0]) / (Point1[1] - Point2[1])
    z3x = Point2[0] - Point2[1] * d_err

    return z3x


def gaze2d_to_gaze3d(gaze2d, zw, P):
    """
    Compute the 3d gaze point in world frame
    """

    u, v = gaze2d[0], gaze2d[1]

    b = (P[2][3] * v - P[1][3]) / (P[1][1] - P[2][1] * v)
    b1 = (P[2][3] * u - P[0][3]) / (P[0][1] - P[2][1] * u)

    a = (P[1][0] - P[2][0] * v) / (P[1][1] - P[2][1] * v)
    a1 = (P[0][0] - P[2][0] * u) / (P[0][1] - P[2][1] * u)

    c = (P[1][2] - P[2][2] * v) / (P[1][1] - P[2][1] * v)
    c1 = (P[0][2] - P[2][2] * u) / (P[0][1] - P[2][1] * u)

    x = (b - b1 - c * zw + c1 * zw) / (a - a1)
    y = b - a * x - c * zw
    # w = P[2][0] * x + P[2][1] * y + P[2][2] * zw + P[2][3]

    return np.array([x, y, zw])


def find_nearest_non_nan(Dmaptranp, x, y):
    """
    Spirally find the nearest non-NAN depth value to represent the matched invalid coordinates
    """
    margin = 30
    Dmap = Dmaptranp.transpose()
    # Spiral search loop
    i = 1
    original_x = deepcopy(x)
    original_y = deepcopy(y)
    while i < 200 & math.isnan(Dmap[x][y]):
        assert not (
            x >= (Dmap.shape[0] - margin)
            or y >= (Dmap.shape[1] - margin)
            or x <= margin
            or y <= margin
        ), "coordinate is out of boundary"
        j = 0
        while j <= i:
            y = y + 1 if i % 2 == 1 else y - 1
            j += 1
            newdepth = Dmap[x][y]
            if not math.isnan(newdepth):
                break
        j = 0
        while j <= i:
            x = x + 1 if i % 2 == 1 else x - 1
            j += 1
            newdepth = Dmap[x][y]
            if not math.isnan(newdepth):
                break
        i += 1
    # assert math.isnan(Dmap[x][y]), "failed to find valid depth"
    if math.isnan(Dmap[x][y]):
        print("failed to find valid data")
        return False, 0, 0
    newdepth = Dmap[x][y]
    newdepth *= 1000  # from meters to millimeters
    # left up corner as origin
    P3D_x = (x - ZEDM_CX) * newdepth / ZEDM_FX  # (x - ZEDM_CX) * newdepth / ZEDM_FX
    P3D_y = (y - ZEDM_CY) * newdepth / ZEDM_FY  # (y - ZEDM_CY) * newdepth / ZEDM_FY
    P3D_z = newdepth
    shift = math.sqrt((original_x - x) ** 2 + (original_y - y) ** 2)

    print("depth is ", newdepth)
    Pt3d = [P3D_x, P3D_y, P3D_z]
    print("3d coord is ", Pt3d)

    return True, P3D_z, shift


def find_average_non_zero(Dmap, x, y, window_size=10):
    if Dmap[x][y] == 0:
        average_depth = 0

        if x >= 1900 or y >= 1060 or x <= 20 or y <= 20:
            print("exceeded the margin")
            return 0

        roi = Dmap[x - window_size : x + window_size, y - window_size : y + window_size]
        average_depth = np.mean(roi) * roi.size / np.count_nonzero(roi)
        return average_depth
    else:
        return Dmap[x][y]


def find_nearest_non_nan_cali(Dmaptranp, x, y, cmtx0):
    """
    Spirally find the nearest non-NAN depth value to represent the matched invalid coordinates
    """
    zed_fx = cmtx0[0, 0]
    zed_fy = cmtx0[1, 1]
    zed_cx = cmtx0[0, 2]
    zed_cy = cmtx0[1, 2]
    margin = 30
    Dmap = Dmaptranp.transpose()
    # Spiral search loop
    i = 1
    original_x = deepcopy(x)
    original_y = deepcopy(y)
    while i < 200 & math.isnan(Dmap[x][y]):
        assert not (
            x >= (Dmap.shape[0] - margin)
            or y >= (Dmap.shape[1] - margin)
            or x <= margin
            or y <= margin
        ), "coordinate is out of boundary"
        j = 0
        while j <= i:
            y = y + 1 if i % 2 == 1 else y - 1
            j += 1
            newdepth = Dmap[x][y]
            if not math.isnan(newdepth):
                break
        j = 0
        while j <= i:
            x = x + 1 if i % 2 == 1 else x - 1
            j += 1
            newdepth = Dmap[x][y]
            if not math.isnan(newdepth):
                break
        i += 1
    # assert math.isnan(Dmap[x][y]), "failed to find valid depth"
    if math.isnan(Dmap[x][y]):
        print("failed to find valid data")
        return False, 0, 0
    newdepth = Dmap[x][y]
    newdepth *= 1000  # from meters to millimeters
    # left up corner as origin
    P3D_x = (x - zed_cx) * newdepth / zed_fx  # (x - ZEDM_CX) * newdepth / ZEDM_FX
    P3D_y = (y - zed_cy) * newdepth / zed_fy  # (y - ZEDM_CY) * newdepth / ZEDM_FY
    P3D_z = newdepth
    shift = math.sqrt((original_x - x) ** 2 + (original_y - y) ** 2)

    print("depth is ", newdepth)
    Pt3d = [P3D_x, P3D_y, P3D_z]
    print("3d coord is ", Pt3d)

    return True, P3D_z, shift
