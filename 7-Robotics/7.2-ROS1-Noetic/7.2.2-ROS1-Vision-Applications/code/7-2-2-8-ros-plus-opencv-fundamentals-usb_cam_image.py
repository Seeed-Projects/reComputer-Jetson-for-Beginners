#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class UsbCamImageNode:
    def __init__(self):
        rospy.init_node("usb_cam_image_node", anonymous=True)

        self.bridge = CvBridge()

        # subscribe to USB camera images
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("USB Camera Image Subscriber Started")

    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # display the image
        cv2.imshow("USB Camera Image", frame)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        UsbCamImageNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
