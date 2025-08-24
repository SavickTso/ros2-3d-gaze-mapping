#!/usr/bin/env python
import copy

# from threading import Lock
import math
import threading
from collections import deque
from threading import Event
from time import sleep

import cv2 as cv
import rclpy
import tobii_g3
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image


class TobiiImageStreamer:
    def __init__(self, gaze2d_node):
        # For converting images OpenCV <-> ROS (sensor_msgs/Image)
        self.cv_bridge = CvBridge()
        self.image_pub = gaze2d_node.create_publisher(Image, "/gaze_tracker/rgb", 1)
        logger = gaze2d_node.get_logger()
        self.logger = logger
        # Open camera with OpenCV VideoCapture:
        rtsp_url = "rtsp://192.168.1.3:8554/live/all"  # false to disable gaze maker
        self.cap = cv.VideoCapture(rtsp_url)
        if not self.cap.set(cv.CAP_PROP_BUFFERSIZE, 1):
            logger.error("cv.CAP_PROP_BUFFERSIZE not supported")

        self.tobii_image = None
        self.img_lock = threading.Lock()

        logger.info("Attempting to open RTSP stream...")
        while not self.cap.isOpened() and rclpy.ok():
            logger.error("Failed to open RTSP stream...")
            gaze2d_node.create_rate(2).sleep()
            self.cap.open(rtsp_url)

        logger.info("OpenCV stream established")

        # Start thread for capturing images:
        self.cam_thread = threading.Thread(target=self.live_streaming)
        self.cam_thread.start()

        logger.info("Image publisher started!")

    def live_streaming(self):
        """
        Reads and publishes images from Tobii as fast as possible
        """
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(frame))
            else:
                self.logger.info("service not available, waiting again...")
                sleep(0.03)


class TobiiGazeStreamer:
    def __init__(self, gaze2d_node, tobii):
        self.tobii = tobii
        self.queue_x = deque()
        self.queue_y = deque()
        self.queue_cand = deque()
        self.MAXSIZE_FILTER = 100
        self.MAXSIZE_gazefix = 3
        self.GAZEFIX_THRESOLD = 50.0
        self.gazefixed_flag = 0
        self.Gaze2dfixed = Point()
        self.gazecandidate = [0, 0]  # Point()
        self.stop_event = Event()
        self.bkg_thread = threading.Thread(target=self.gazefix, args=())
        self.bkg_thread.daemon = True
        self.bkg_thread.start()

        self.pub_realtime = gaze2d_node.create_publisher(
            Point, "Gaze2d_realtime_Pub", 10
        )
        self.pub_fixed = gaze2d_node.create_publisher(Point, "Gaze2d_fixed_Pub", 10)
        self.Gaze2d = Point()

        # bkg_thread.join()
        print("2d gaze stream started!!!")

    # background task to get fixed 2d gazes
    def gazefix(self):
        # run until the event is set
        while not self.stop_event.is_set() and rclpy.ok():
            if self.gazecandidate[0] > 0 and self.gazecandidate[1] > 0:
                if len(self.queue_cand) >= self.MAXSIZE_gazefix:
                    self.queue_cand.popleft()
                    self.queue_cand.append(self.gazecandidate)
                    cand_1 = self.queue_cand.pop()
                    cand_2 = self.queue_cand.pop()
                    cand_3 = self.queue_cand.pop()
                    dist_12 = math.dist(cand_1, cand_2)
                    dist_13 = math.dist(cand_1, cand_3)
                    dist_23 = math.dist(cand_2, cand_3)
                    dist_mean = (dist_12 + dist_13 + dist_23) / 3
                    # print(dist_mean)
                    if dist_mean < self.GAZEFIX_THRESOLD:
                        self.Gaze2dfixed.x = cand_2[0]
                        self.Gaze2dfixed.y = cand_2[1]
                        self.gazefixed_flag = 1
                        print("2d gaze fixed !")
                        print(self.Gaze2dfixed)
                    else:
                        # self.Gaze2dfixed = [0,0]
                        self.gazefixed_flag = 0
                else:
                    self.queue_cand.append(self.gazecandidate)
            # print(queue_cand)
            # run every 3 seconds
            sleep(0.3)
        print("Background done")

    def gaze2dfilter(self):
        if len(self.queue_x) > self.MAXSIZE_FILTER:
            self.queue_x.popleft()
            self.queue_y.popleft()
        self.queue_x.append(self.Gaze2d.x)
        self.queue_y.append(self.Gaze2d.y)
        self.gazecandidate[0], self.gazecandidate[1] = sum(self.queue_x) / len(
            self.queue_x
        ), sum(self.queue_y) / len(self.queue_y)

    def gaze2dupdate(self):
        if not self.tobii.connected:
            self.tobii.connect()
        self.tobii.send_action("rudimentary", "keepalive")
        data = self.tobii.gazedataRT
        if "data" in data:
            if "gaze2d" in data["data"]:
                xcoor = int(1920 * data["data"]["gaze2d"][0])  # from left
                ycoor = int(1080 * data["data"]["gaze2d"][1])  # from top
                if xcoor < 0 or ycoor < 0:
                    return []  # False
                # print("the xy coordinate of the gaze point in the camera scene is: [{0}, {1} ] at time of {2}".format(xcoor, ycoor, data['timestamp']))
                gaze2d = [xcoor, ycoor]
            else:
                gaze2d = (
                    []
                )  # gaze2dupdate(tobii)#keep running this until the gaze data is retrieved
        else:
            gaze2d = (
                []
            )  # gaze2dupdate(tobii)#keep running this until the gaze data is retrieved

        return gaze2d

    def gazeRealtimePublisher(self):
        # rospy.loginfo(self.Gaze2d)
        self.pub_realtime.publish(self.Gaze2d)
        # self.rate.sleep()

    def gazeFixedPublisher(self):
        self.pub_fixed.publish(self.Gaze2dfixed)
        # self.rate.sleep()


def main():
    rclpy.init()
    tobii = tobii_g3.G3Client("192.168.1.3")
    print(tobii.discover_g3())
    tobii.connect()
    tobii.set_gaze_overlay()

    gaze2d_node = rclpy.create_node("gaze_stream")
    imagepb = TobiiImageStreamer(gaze2d_node)
    gazepb = TobiiGazeStreamer(gaze2d_node, tobii)
    while rclpy.ok():
        try:
            # imagepb.TobiiLiveStream()
            gaze2d = gazepb.gaze2dupdate()
            if gaze2d == []:
                continue
            else:
                # gaze2d_node.get_logger().info(
                #     "gaze2d[0] value is {} ".format(gaze2d[0])
                # )
                gazepb.Gaze2d.x = float(gaze2d[0])
                gazepb.Gaze2d.y = float(gaze2d[1])
                gazepb.gazeRealtimePublisher()
                gazepb.gaze2dfilter()
            if gazepb.gazefixed_flag == 1:
                gazepb.gazeFixedPublisher()
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")
            break
    gaze2d_node.destroy_node()
    tobii.disconnect()
    imagepb.cam_thread.join()
    imagepb.cap.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
