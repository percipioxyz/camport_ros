#include "percipio_camera/percipio_depth_algorithm.h"
namespace percipio
{

void DepthTimeDomainMgr::reset(const int frame)
{
  std::unique_lock<std::mutex> lock(_mutex);
  m_image_num = frame;
  images.clear();
}

void DepthTimeDomainMgr::add_frame(const TY_IMAGE_DATA& img)
{
  std::unique_lock<std::mutex> lock(_mutex);
  while(images.size() >= m_image_num) {
    images.erase(images.begin());
  }
  images.push_back(image_data(img));
}

bool DepthTimeDomainMgr::do_time_domain_process(TY_IMAGE_DATA& img)
{
  std::unique_lock<std::mutex> lock(_mutex);
  if(images.size() < m_image_num) {
    return false;
  }

  std::vector<TY_IMAGE_DATA> imgs;
  for(size_t i = 0; i < images.size(); i++) {
    imgs.push_back( images[i].toTyImage(TY_COMPONENT_DEPTH_CAM));
  }

  static DepthEnhenceParameters def_params_ = DepthEnhenceParameters_Initializer;
  TY_STATUS ret = TYDepthEnhenceFilter(&imgs[0], imgs.size(), nullptr, &img, &def_params_);
  return ret == TY_STATUS_OK ? true : false;
}

}