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

#include "percipio_camera/percipio_device_manager.h"
#include "percipio_camera/percipio_convert.h"
#include "percipio_camera/percipio_device.h"
#include "percipio_camera/percipio_exception.h"

#include <boost/make_shared.hpp>

#include <ros/ros.h>

#include <set>
#include <string>

#include "TYApi.h"

namespace percipio_wrapper
{

class PercipioDeviceInfoComparator
{
public:
  bool operator()(const PercipioDeviceInfo& di1, const PercipioDeviceInfo& di2) const
  {
    return (di1.uri_.compare(di2.uri_) < 0);
  }
};

typedef std::set<PercipioDeviceInfo, PercipioDeviceInfoComparator> DeviceSet;

class PercipioDeviceListener : public percipio::DeviceConnectedListener,
                               public percipio::DeviceDisconnectedListener
{
public:
  PercipioDeviceListener() : 
        percipio::DeviceConnectedListener(),
        percipio::DeviceDisconnectedListener()
  {
    percipio::Percipio::addDeviceConnectedListener(this);
    percipio::Percipio::addDeviceDisconnectedListener(this);
    //percipio::Percipio::addDeviceStateChangedListener(this);

    // get list of currently connected devices
    percipio::Array<percipio::DeviceInfo> device_info_list;
    percipio::Percipio::enumerateDevices(&device_info_list);
    for (int i = 0; i < device_info_list.getSize(); ++i)
    {
      //onDeviceConnected(&device_info_list[i]);
      const PercipioDeviceInfo device_info_wrapped = percipio_convert(&device_info_list[i]);

      // make sure it does not exist in set before inserting
      device_set_.erase(device_info_wrapped);
      device_set_.insert(device_info_wrapped);
    }
  }

  ~PercipioDeviceListener()
  {
    percipio::Percipio::removeDeviceConnectedListener(this);
    percipio::Percipio::removeDeviceDisconnectedListener(this);
  }

  virtual void onDeviceConnected(const percipio::DeviceInfo* pInfo)
  {
    boost::mutex::scoped_lock l(device_mutex_);

    const PercipioDeviceInfo device_info_wrapped = percipio_convert(pInfo);

    if(strlen(pInfo->getIp()))
      ROS_INFO("Device \"%s\" IP:%s found.", pInfo->getUri(), pInfo->getIp());
    else
      ROS_INFO("Device \"%s\" found.", pInfo->getUri());

    // make sure it does not exist in set before inserting
    device_set_.erase(device_info_wrapped);
    device_set_.insert(device_info_wrapped);
  }

  virtual void onDeviceDisconnected(const percipio::DeviceInfo* pInfo)
  {
    boost::mutex::scoped_lock l(device_mutex_);

    if(strlen(pInfo->getIp()))
      ROS_WARN("Device \"%s\" IP:%s disconnected\n", pInfo->getUri(), pInfo->getIp());
    else
      ROS_WARN("Device \"%s\" disconnected\n", pInfo->getUri());

    const PercipioDeviceInfo device_info_wrapped = percipio_convert(pInfo);
    device_set_.erase(device_info_wrapped);

    //kill node
    //system("rosnode kill /camera/driver");
  }

  boost::shared_ptr<std::vector<std::string> > getConnectedDeviceURIs()
  {
    percipio::Array<percipio::DeviceInfo> device_info_list;
    percipio::Percipio::enumerateDevices(&device_info_list);
    for (int i = 0; i < device_info_list.getSize(); ++i)
    {
      onDeviceConnected(&device_info_list[i]);
    }

    boost::mutex::scoped_lock l(device_mutex_);

    boost::shared_ptr<std::vector<std::string> > result = boost::make_shared<std::vector<std::string> >();

    result->clear();
    result->reserve(device_set_.size());

    std::set<PercipioDeviceInfo, PercipioDeviceInfoComparator>::const_iterator it;
    std::set<PercipioDeviceInfo, PercipioDeviceInfoComparator>::const_iterator it_end = device_set_.end();

    for (it = device_set_.begin(); it != it_end; ++it) {
      result->push_back(it->uri_);
      if(strlen(it->ip_.c_str()))
        result->push_back(it->ip_);
    }

    return result;
  }

  boost::shared_ptr<std::vector<PercipioDeviceInfo> > getConnectedDeviceInfos()
  {
    boost::mutex::scoped_lock l(device_mutex_);
    boost::shared_ptr<std::vector<PercipioDeviceInfo> > result = boost::make_shared<std::vector<PercipioDeviceInfo> >();
    result->reserve(device_set_.size());
    DeviceSet::const_iterator it;
    DeviceSet::const_iterator it_end = device_set_.end();

    for (it = device_set_.begin(); it != it_end; ++it)
      result->push_back(*it);

    return result;
  }

  std::size_t getNumOfConnectedDevices()
  {
    boost::mutex::scoped_lock l(device_mutex_);

    return device_set_.size();
  }

  boost::mutex device_mutex_;
  DeviceSet device_set_;
};

//////////////////////////////////////////////////////////////////////////

boost::shared_ptr<PercipioDeviceManager> PercipioDeviceManager::singelton_;

PercipioDeviceManager::PercipioDeviceManager()
{
  TY_STATUS rc = percipio::Percipio::initialize();
  if (rc != TY_STATUS_OK)
      THROW_PERCIPIO_EXCEPTION("Initialize failed\n%s\n", percipio::Percipio::getExtendedError(rc));

  device_listener_ = boost::make_shared<PercipioDeviceListener>();
}

PercipioDeviceManager::~PercipioDeviceManager()
{
}

boost::shared_ptr<PercipioDeviceManager> PercipioDeviceManager::getSingelton()
{
  if (singelton_.get()==0)
    singelton_ = boost::make_shared<PercipioDeviceManager>();

  return singelton_;
}

boost::shared_ptr<std::vector<PercipioDeviceInfo> > PercipioDeviceManager::getConnectedDeviceInfos() const
{
return device_listener_->getConnectedDeviceInfos();
}

boost::shared_ptr<std::vector<std::string> > PercipioDeviceManager::getConnectedDeviceURIs() const
{
  return device_listener_->getConnectedDeviceURIs();
}

std::size_t PercipioDeviceManager::getNumOfConnectedDevices() const
{
  return device_listener_->getNumOfConnectedDevices();
}

std::string PercipioDeviceManager::getSerial(const std::string& Uri) const
{
  percipio::Device percipio_device;
  std::string ret;

  // we need to open the device to query the serial number
  if (Uri.length() > 0 && percipio_device.open(Uri.c_str()) == TY_STATUS_OK)
  {
    ret = percipio_device.getDeviceInfo().getUri();
    // close the device again
    percipio_device.close();
  }
  else
  {
    THROW_PERCIPIO_EXCEPTION("Device open failed: %s", Uri.c_str());
  }
  return ret;
}

boost::shared_ptr<PercipioDevice> PercipioDeviceManager::getAnyDevice()
{
  return boost::make_shared<PercipioDevice>("");
}
boost::shared_ptr<PercipioDevice> PercipioDeviceManager::getDevice(const std::string& device_URI, const bool reconnection)
{
  return boost::make_shared<PercipioDevice>(device_URI, reconnection);
}


std::ostream& operator << (std::ostream& stream, const PercipioDeviceManager& device_manager) {

  boost::shared_ptr<std::vector<PercipioDeviceInfo> > device_info = device_manager.getConnectedDeviceInfos();

  std::vector<PercipioDeviceInfo>::const_iterator it;
  std::vector<PercipioDeviceInfo>::const_iterator it_end = device_info->end();

  for (it = device_info->begin(); it != it_end; ++it)
  {
    stream << "Uri: " << it->uri_ << " (Vendor: " << it->vendor_ <<
                                     ", Name: " << it->name_ <<
                                     ", Vendor ID: " << it->vendor_id_ <<
                                     ", Product ID: " << it->product_id_ <<
                                      ")" << std::endl;
  }

  return stream;
}


} //namespace percipio_wrapper
