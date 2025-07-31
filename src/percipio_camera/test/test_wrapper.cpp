#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_device.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>

#include <iostream>

using namespace std;
using namespace percipio_wrapper;

int ir_counter_ = 0;
int color_counter_ = 0;
int depth_counter_ = 0;

void IRCallback(sensor_msgs::ImagePtr image)
{
  ++ir_counter_;
}

void ColorCallback(sensor_msgs::ImagePtr image)
{
  ++color_counter_;
}

void DepthCallback(sensor_msgs::ImagePtr image)
{
  ++depth_counter_;
}

int main()
{
  PercipioDeviceManager device_manager;

  std::cout << device_manager;

  boost::shared_ptr<std::vector<std::string> > device_uris = device_manager.getConnectedDeviceURIs();

  BOOST_FOREACH(const std::string& uri, *device_uris)
  {
    boost::shared_ptr<PercipioDevice> device = device_manager.getDevice(uri);
    std::cout << *device;

    device->setIRFrameCallback(boost::bind(&IRCallback, _1));
    device->setColorFrameCallback(boost::bind(&ColorCallback, _1));
    device->setDepthFrameCallback(boost::bind(&DepthCallback, _1));

    ir_counter_ = 0;
    color_counter_ = 0;
    depth_counter_ = 0;

    device->startColorStream();
    device->startDepthStream();

    boost::this_thread::sleep(boost::posix_time::milliseconds(6000));

    device->stopAllStreams();

    std::cout<<std::endl;

    std::cout<<"Number of called to IRCallback: "<< ir_counter_ << std::endl;
    std::cout<<"Number of called to ColorCallback: "<< color_counter_ << std::endl;
    std::cout<<"Number of called to DepthCallback: "<< depth_counter_ << std::endl;
  }


  return 0;
}
