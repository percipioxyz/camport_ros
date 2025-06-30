/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-10-19 17:10:35
 * @LastEditors: zxy
 * @LastEditTime: 2023-12-14 10:15:46
 */

#include "percipio_camera/PercipioConfig.h"
#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_driver.h"
#include <dynamic_reconfigure/server.h>

void callback(percipio_camera::PercipioConfig &config, uint32_t level)
{
}

int main(int argc, char **argv){
  ros::init(argc, argv, "percipio_camera");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  percipio_wrapper::PercipioDriver drv(n, pnh);
  //dynamic_reconfigure::Server<percipio_camera::PercipioConfig> server;
	//dynamic_reconfigure::Server<percipio_camera::PercipioConfig>::CallbackType f;
  //f = boost::bind(&callback, _1, _2);
  //server.setCallback(f);

  ros::spin();

  return 0;
}
