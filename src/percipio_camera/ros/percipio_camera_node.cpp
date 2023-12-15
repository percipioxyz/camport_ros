/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-10-19 17:10:35
 * @LastEditors: zxy
 * @LastEditTime: 2023-12-14 10:15:46
 */
/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Julius Kammerl (jkammerl@willowgarage.com)
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
