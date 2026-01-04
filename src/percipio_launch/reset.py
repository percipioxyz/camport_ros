#!/usr/bin/env python
"""
reset.py
Client example for reset Percipio camera
Usage:
    python reset.py [camera_name]
Examples:
    python reset.py camera
"""

import rospy
import sys
from std_msgs.msg import Empty

def reset(camera_name="camera"):
    """
    Send reset signals to specified camera
    
    Args:
        camera_name: Camera name
    """
    try:
        # Initialize ROS node
        rospy.init_node('device_reset_client', anonymous=True)
        
        topic_name = f"/{camera_name}/reset"
        
        # Create publisher
        reset_pub = rospy.Publisher(topic_name, Empty, queue_size=5)
        
        # Wait for publisher connections (optional)
        rospy.sleep(0.5)
        
        # Create and publish empty message
        reset_msg = Empty()
        reset_pub.publish(reset_msg)
        print(f"✓ Successfully sent reset signals")
        
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except Exception as e:
        print(f"Error sending reset signals: {e}")

if __name__ == "__main__":
    # Get camera name from command line arguments
    camera_name = "camera"  # Default value
    
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]
    
    print(f"=== Percipio Camera Reset Client ===")
    print(f"Camera name: {camera_name}")
    reset(camera_name)