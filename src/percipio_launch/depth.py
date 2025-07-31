#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/depth/image", Image, self.callback)
        #self.info_sub = message_filters.Subscriber('/camera/depth/camera_info', CameraInfo)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            #print("Received camera info:", info_msg)
            cv2.imshow("Depth window", cv_image)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr("convert failed: %s", e)

def main():
    rospy.init_node('depth_converter')
    ic = ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
