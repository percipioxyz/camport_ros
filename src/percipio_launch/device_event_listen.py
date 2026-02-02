#!/usr/bin/env python
import rospy
import sys

from std_msgs.msg import String

def device_event_callback(msg):
    """
    Device event callback function
    """
    rospy.loginfo(f"Device Event: {msg.data}")

def listener(camera_name: str = "camera") -> None:
    """
    Listener main function
    """
    # Initialize ROS node
    rospy.init_node('percipio_device_event_listener', anonymous=True)
    
    # Construct topic name
    topic_name = f'/{camera_name}/PercipioDeviceEvent'
    
    rospy.loginfo(f"Listening to device events on topic: {topic_name}")
    
    # Subscribe to device event topic
    rospy.Subscriber(topic_name, String, device_event_callback)
    
    # Keep the program running until shutdown
    rospy.spin()

if __name__ == '__main__':
    camera_name = "camera"
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]

    try:
        listener(camera_name)
    except rospy.ROSInterruptException:
        pass