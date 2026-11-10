#!/usr/bin/env python3

import socket

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

JPEG_START = b"\xff\xd8"
JPEG_END = b"\xff\xd9"
MAX_JPEG_BYTES = 2 * 1024 * 1024


def publish_stream(sock, publisher, bridge, frame_id):
    buffer = bytearray()

    while not rospy.is_shutdown():
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            continue

        if not chunk:
            raise ConnectionError("Camera stream closed")

        buffer.extend(chunk)

        while True:
            start = buffer.find(JPEG_START)
            if start < 0:
                buffer[:] = buffer[-1:]
                break
            if start:
                del buffer[:start]

            end = buffer.find(JPEG_END, 2)
            if end < 0:
                if len(buffer) > MAX_JPEG_BYTES:
                    rospy.logwarn_throttle(10, "Dropping oversized JPEG frame")
                    del buffer[:2]
                    continue
                break

            jpg = bytes(buffer[:end + 2])
            del buffer[:end + 2]
            frame = cv2.imdecode(
                np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                rospy.logwarn_throttle(10, "Could not decode camera frame")
                continue

            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            publisher.publish(msg)


def main():
    rospy.init_node("pi_camera_bridge")
    host = rospy.get_param("~host", "172.17.0.1")
    port = rospy.get_param("~port", 8888)
    topic = rospy.get_param("~topic", "/pi_cam/image_raw")
    frame_id = rospy.get_param("~frame_id", "pi_cam")
    publisher = rospy.Publisher(topic, Image, queue_size=1)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        try:
            with socket.create_connection((host, port), timeout=5) as sock:
                sock.settimeout(5)
                rospy.loginfo("Connected to camera stream at %s:%s", host, port)
                publish_stream(sock, publisher, bridge, frame_id)
        except OSError as error:
            if not rospy.is_shutdown():
                rospy.logerr_throttle(10, "Camera stream unavailable: %s", error)
                rospy.sleep(3)


if __name__ == "__main__":
    main()
