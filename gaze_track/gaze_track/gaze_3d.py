#!/usr/bin/env python3
import math
import os
import time
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from gaze_track.utils import *


class GazeSearcher:
    def __init__(self, node, init_z=None):
        self.bridge = CvBridge()
        self.node = node  # rclpy.init_node("gaze3d_node")
        self.image_pub = node.create_publisher(Image, "/gaze_tracker/rbg_overlay", 1)
        current_dir = os.getcwd()
        self.node.get_logger().info("current folder is {}".format(current_dir))
        package_path = get_package_share_directory("gaze_track")
        self.node.get_logger().info("package path is {}".format(package_path))
        self.P0 = get_projection_matrix(0, package_path)
        self.P1 = get_projection_matrix(1, package_path)

        # Handle init_z parameter
        if init_z is not None:
            self.init_z = init_z
            self.use_fixed_init_z = True
            self.node.get_logger().info("Using provided init_z: {}".format(init_z))
        else:
            self.init_z = 6000  # default fallback value
            self.use_fixed_init_z = False
            self.node.get_logger().info("Will use smallest valid depth value as init_z")

        self.errorlist = []

    def depthmapupdate(self):
        flag = False
        if rclpy.ok():
            flag, m = wait_for_message(
                Image,
                self.node,
                "/zedm/zed_node/depth/depth_registered",
                0.03,
            )
            # self.node.get_logger().info(
            #     "depth image received with shape ({}, {})".format(
            #         m[1].height, m[1].width
            #     )
            # )
            if flag:
                self.node.get_logger().info("depth image successfully received")
                self.depth_image = self.bridge.imgmsg_to_cv2(m, "32FC1")
                self.depth_array = np.array(self.depth_image, dtype=np.float32)
            else:
                self.node.get_logger().info("Failed to get depth image ")

    def zedrgbupdate(self):
        flag = False
        if rclpy.ok():
            flag, m = wait_for_message(
                Image,
                self.node,
                "/zedm/zed_node/left/image_rect_color",
                0.3,
            )
            if flag:
                self.node.get_logger().info("rgb image successfully received")
                self.rgb_image = self.bridge.imgmsg_to_cv2(m)
            else:
                self.node.get_logger().info("Failed to get rgb image ")

    def gaze2dupdate(self):
        if rclpy.ok():
            self.node.get_logger().info("waiting 2d gaze point")
            tic = time.time()
            m = wait_for_message(
                Point,
                self.node,
                "/Gaze2d_realtime_Pub",
                0.01,
            )

            if m[0]:
                self.gaze2d = [m[1].x, m[1].y]
                self.node.get_logger().info(
                    "2d Point received and updated, spent {} sec".format(
                        time.time() - tic
                    )
                )
            else:
                self.node.get_logger().info("Failed to get 2d gaze ")

    def update_init_z_from_smallest_depth(self):
        if not self.use_fixed_init_z and hasattr(self, "depth_array"):
            depth_array = self.depth_array
            valid_depths = depth_array[~np.isnan(depth_array) & (depth_array > 0)]

            if len(valid_depths) > 0:
                smallest_depth = np.min(valid_depths)
                self.init_z = smallest_depth
                self.node.get_logger().info(
                    "Updated init_z to smallest valid depth: {} mm".format(self.init_z)
                )
            else:
                self.node.get_logger().warn(
                    "No valid depths found, keeping default init_z: {}".format(
                        self.init_z
                    )
                )

    def search(self, max_iter=5):
        self.update_init_z_from_smallest_depth()

        flag, gaze3d_inferred, new_depth, zed2d = self.world_to_zed3d(
            self.init_z, self.P1
        )
        if not flag:
            return gaze3d_inferred, zed2d, flag

        # Start Newton's method
        self.gaze3d, self.zed2d, flag = self.newton_search_iter(
            gaze3d_inferred, new_depth, max_iter
        )
        if not flag:
            return gaze3d_inferred, zed2d, flag
        print("gaze 2d coordinates in zed are ", self.zed2d)
        print("gaze 3d coordinates are ", self.gaze3d)

        return self.gaze3d, self.zed2d, flag

    def newton_search_iter(self, gaze3d_inferred, new_depth, max_iter):
        init_z = self.init_z
        P1 = self.P1
        point_old = np.array([init_z, new_depth - init_z])
        z_new = init_z + 400
        iter_num = 0
        flag = True

        # while z_new - Zed3d[2] < 0 and iter_num < max_iter:
        while iter_num < max_iter and flag:
            flag, gaze3d_inferred, new_depth, zed2d = self.world_to_zed3d(z_new, P1)
            point_new = np.array([z_new, new_depth - z_new])
            z_new = newton3d_depth(point_old, point_new)
            print("depth computed in this iteration is ", z_new)
            if math.isnan(z_new) or z_new <= 0.0 or math.isinf(z_new):
                z_new = point_new[0]
                break
            point_old = point_new
            iter_num += 1
        print("iterated {} times in newton's method".format(iter_num))

        # return the found Gazepoint in 3d coord
        return gaze3d_inferred, zed2d, flag

    def world_to_zed3d(self, z=300, P=None):
        if P is None:
            P = self.P1
        depth_array = self.depth_array
        gaze3d_inferred = gaze2d_to_gaze3d([self.gaze2d[0], self.gaze2d[1]], z, P)
        zedx, zedy = get_projected_cam(
            int(gaze3d_inferred[0]),
            int(gaze3d_inferred[1]),
            int(gaze3d_inferred[2]),
            self.P0,
        )
        self.node.get_logger().info("zed2d is {}, {}".format(zedx, zedy))
        zed2d = [int(zedx), int(zedy)]

        # self.node.get_logger().info(
        #     "the deviation caused by non-nan filter is :{}, {}".format(
        #         zed2d[0] - nearest_x, zed2d[1] - nearest_y
        #     )
        # )
        if 0 <= zed2d[0] <= 1919 and 0 <= zed2d[1] <= 1079:
            # new_depth = depth_array[zed2d[0]][zed2d[1]]
            # self.nearest_xy = [nearest_x, nearest_y]
            _, new_depth, _ = find_nearest_non_nan(depth_array, zed2d[0], zed2d[1])
            return True, gaze3d_inferred, new_depth, zed2d

        print("the transformed coord is wrong")
        new_depth = z
        return False, gaze3d_inferred, new_depth, zed2d

    def pointdisplay(self):
        image_ocv = self.rgb_image
        cv2.circle(
            image_ocv,
            (self.zed2d[0], self.zed2d[1]),
            20,
            (255, 100, 100),
            cv2.FILLED,
            5,
            shift=0,
        )
        cv2.putText(
            image_ocv,
            "World Coordinate: ({}, {}, {})".format(
                round(self.gaze3d[0], 2),
                round(self.gaze3d[1], 2),
                round(self.gaze3d[2], 2),
            ),
            (self.zed2d[0] - 80, self.zed2d[1] + 40),
            cv2.FONT_HERSHEY_COMPLEX_SMALL,
            1,
            (0, 255, 0),
            3,
        )
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_ocv))

        return True


def Gaze3d_Publisher(gaze3d, gazepub):
    Gaze3d = Point()
    Gaze3d.x = gaze3d[0]
    Gaze3d.y = gaze3d[1]
    Gaze3d.z = gaze3d[2]

    gazepub.publish(Gaze3d)


def SearchIt(node, init_z=None):
    searcher = GazeSearcher(node, init_z)
    # SYS_RETRIEVE = True
    zedcoor_list = []
    gaze3dpub = node.create_publisher(Point, "Gaze3d_pub", 10)

    while rclpy.ok():
        try:
            # make sure got depthmap and 2d gaze
            searcher.gaze2dupdate()
            searcher.depthmapupdate()
            while (not hasattr(searcher, "gaze2d")) or (
                not hasattr(searcher, "depth_array")
            ):
                # if not rclpy.ok():
                #     break
                searcher.gaze2dupdate()
                searcher.depthmapupdate()
                node.get_logger().warn("depth data or gaze data is missing, passed")

            start_time = time.time()
            gaze3d, zed2d, valid_flag = searcher.search()

            if not valid_flag:
                continue
            searcher.node.get_logger().info(
                "searching time spent is:  {}".format(time.time() - start_time)
            )
            node.get_logger().info(
                "publishing gaze3d{},{},{}".format(gaze3d[0], gaze3d[1], gaze3d[2])
            )
            # searcher.zedrgbupdate() #for maximum frequency
            # searcher.pointdisplay()
            Gaze3d_Publisher(gaze3d, gaze3dpub)
        except KeyboardInterrupt:
            # exit the tracking
            searcher.node.get_logger().info("Gaze searching exited")
    searcher.close()


def main():
    rclpy.init()
    node = Node("gaze3d_node")
    SearchIt(node, init_z=6000)


if __name__ == "__main__":
    main()
