#!/usr/bin/env python
import rospy
import sys
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:
    def __init__(self,camera_name:str="camera"):
        self.bridge = CvBridge()
        self.camera_name = camera_name
        self.image_sub = rospy.Subscriber(f'/{camera_name}/depth/image', Image, self.callback)
        #self.info_sub = message_filters.Subscriber('/camera/depth/camera_info', CameraInfo)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            #print("Received camera info:", info_msg)
            window_name = f"Depth window - {self.camera_name}"
            cv2.imshow(window_name, cv_image)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            rospy.logerr("convert failed: %s", e)

def main():
    camera_name = "camera"  # default camera name
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]

    rospy.init_node(f'depth_converter_{camera_name}')
    ic = ImageConverter(camera_name)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
