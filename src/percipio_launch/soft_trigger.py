#!/usr/bin/env python
"""
soft_trigger.py
Client example for sending soft trigger signals to Percipio camera
Usage:
    python soft_trigger.py [camera_name] [trigger_count]
Examples:
    python soft_trigger.py camera 1    # Send 1 trigger to camera 'camera'
    python soft_trigger.py camera 5    # Send 5 triggers to camera 'camera' with 1-second intervals
"""

import rospy
import sys
import time
from std_msgs.msg import Empty

def send_soft_trigger(camera_name="camera", count=1, interval=1.0):
    """
    Send soft trigger signals to specified camera
    
    Args:
        camera_name: Camera name (consistent with camera parameter in launch file)
        count: Number of triggers
        interval: Trigger interval in seconds
    """
    try:
        # Initialize ROS node
        rospy.init_node('soft_trigger_client', anonymous=True)
        
        # Construct topic name (consistent with soft_trigger_topic parameter in launch file)
        topic_name = f"/{camera_name}/soft_trigger"
        
        # Create publisher
        trigger_pub = rospy.Publisher(topic_name, Empty, queue_size=10)
        
        # Wait for publisher connections (optional)
        rospy.sleep(0.5)
        
        # Send specified number of trigger signals
        for i in range(count):
            if rospy.is_shutdown():
                break
                
            # Create and publish empty message
            trigger_msg = Empty()
            trigger_pub.publish(trigger_msg)
            
            print(f"[{time.strftime('%H:%M:%S')}] Sent trigger {i+1}/{count} to topic: {topic_name}")
            
            if i < count - 1:  # Wait interval if not the last trigger
                time.sleep(interval)
                
        print(f"✓ Successfully sent {count} soft trigger signals")
        
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except Exception as e:
        print(f"Error sending trigger signals: {e}")

if __name__ == "__main__":
    # Get camera name and trigger count from command line arguments
    camera_name = "camera"  # Default value
    count = 1               # Default value
    
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            count = int(sys.argv[2])
        except ValueError:
            print("Error: Trigger count must be an integer")
            sys.exit(1)
    
    print(f"=== Percipio Camera Soft Trigger Client ===")
    print(f"Camera name: {camera_name}")
    print(f"Trigger count: {count}")
    print("=" * 30)
    
    send_soft_trigger(camera_name, count)