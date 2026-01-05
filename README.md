1. build ros

$catkin_make -DCMAKE_BUILD_TYPE=Release

2. config system env

$echo "source ~/camport_ros/devel/setup.bash" >> ~/.bashrc

$source ~/.bashrc

Please refer to https://doc.percipio.xyz/cam/latest/getstarted/sdk-ros1-compile.html for more detais.

3. list devices
rosrun percipio_camera list_devices


4. Set the IP info of the network device
rosrun percipio_camera network_ip_config


5. Example Program Description
    depth.py: Subscribes to the depth map of the specified camera.
    color.py: Subscribes to the color image of the specified camera.
    cloud.py: Subscribes to the point cloud data of the specified camera.
    depthcloud.py: Subscribes to the colored point cloud data of the specified camera.
    parameters.xml: Used for static configuration of camera parameters during startup.
    dynamic_config.py: Used for dynamically setting camera parameters during operation.
    reset.py: Used to control camera reset (Note: The reset logic may vary among different camera models).
    soft_trigger.py: Used to send a soft trigger signal to cameras operating in trigger mode, initiating the camera's data stream output.
    device_event_listen.py: Used to listen for camera event messages (device offline events and successful reconnection events when automatic offline reconnection mode is enabled).


   
