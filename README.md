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
