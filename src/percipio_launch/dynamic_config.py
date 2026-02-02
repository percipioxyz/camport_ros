#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Percipio Camera Dynamic Configuration Client
============================================

This script publishes XML configuration messages to dynamically reconfigure
Percipio camera parameters through ROS topic interface.
"""

import rospy
import sys
from std_msgs.msg import String


def configure_camera(camera_name: str = "camera") -> None:
    """
    Publish XML configuration messages to reconfigure Percipio camera parameters.

    Args:
        camera_name: ROS namespace for the camera device (default: "camera")

    Process:
        1. Initialize ROS node
        2. Create publisher for dynamic configuration topic
        3. Construct XML configuration with camera parameters
        4. Publish configuration message to camera

    Raises:
        rospy.ROSInterruptException: If ROS node is interrupted (e.g., Ctrl+C)
        Exception: For any other unexpected errors during configuration
    """
    
    # Initialize ROS node for camera configuration
    rospy.init_node('camera_configurator', anonymous=True)
    
    # Construct topic name for dynamic configuration based on camera namespace
    topic_name = f"/{camera_name}/dynamic_config"
    rospy.loginfo(f"Publishing to topic: {topic_name}")
    
    # Create ROS publisher for String messages on dynamic configuration topic
    # Queue size of 10 ensures minimal message loss during high-frequency publishing
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    
    # Wait for publisher to establish connection with subscribers
    # This delay ensures the publisher is ready before sending messages
    rospy.sleep(1)
    
    # XML configuration containing camera parameters
    # The XML format maintains parameter order and supports repeated elements
    # The following XML is used to configure AEC ROI(10, 10, 30, 30)
    xml_config = '''
    <source name="Texture">
        <feature name="ExposureAuto">1</feature>
        <feature name="AutoFunctionAOIOffsetX">10</feature>
        <feature name="AutoFunctionAOIOffsetY">10</feature>
        <feature name="AutoFunctionAOIWidth">30</feature>
        <feature name="AutoFunctionAOIHeight">30</feature>
    </source>'''
    
    # Create ROS String message with XML configuration
    config_msg = String()
    config_msg.data = xml_config
    
    # Publish configuration message to dynamic configuration topic
    pub.publish(config_msg)
    rospy.loginfo("XML configuration published successfully")
    
    # Log a preview of the XML being sent for debugging purposes
    xml_preview = xml_config[:100] + "..." if len(xml_config) > 100 else xml_config
    rospy.loginfo(f"XML configuration preview: {xml_preview}")
    
    # Brief pause to ensure message transmission completes before node shutdown
    rospy.sleep(0.5)


def main() -> None:
    """
    Main execution block for the camera configuration client.
    
    Usage:
        python dynamic_config.py [camera_name]
    
    Arguments:
        camera_name (optional): ROS namespace of the camera to configure
                               Default: "camera"
    
    Examples:
        # Configure default camera with basic parameters
        python dynamic_config.py
        
        # Configure camera named "camera_1" with basic parameters
        python dynamic_config.py camera_1
    """
    
    # Default camera name if no command-line argument provided
    camera_name = "camera"
    
    # Parse command line arguments
    # sys.argv[0] is the script name, sys.argv[1] would be the first argument
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]
    
    # Display startup information
    print("=" * 50)
    print("Percipio Camera XML Configuration Client")
    print("=" * 50)
    print(f"Camera namespace: {camera_name}")
    print(f"Dynamic config topic: /{camera_name}/dynamic_config")
    print("=" * 50)
    
    try:
        # Execute camera configuration function
        configure_camera(camera_name)
        rospy.loginfo("Camera configuration completed successfully")
        
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exceptions (e.g., Ctrl+C)
        rospy.logwarn("ROS node interrupted before completion")
        
    except Exception as e:
        # Catch and display any unexpected errors
        rospy.logerr(f"Unexpected error during configuration: {e}")
        sys.exit(1)
        
    finally:
        rospy.loginfo("Camera configuration client finished execution")


if __name__ == "__main__":
    main()
