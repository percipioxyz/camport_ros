/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 14:00:15
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-04 14:02:22
 */

#ifndef PERCIPIO_TIME_FILTER_H_
#define PERCIPIO_TIME_FILTER_H_

#include <deque>

#include "TYApi.h"

namespace percipio_wrapper
{

class PercipioTimerFilter
{
public:
  PercipioTimerFilter(std::size_t filter_len);
  virtual ~PercipioTimerFilter();

  void addSample(double sample);

  double getMedian();
  double getMovingAvg();

  void clear();

private:
  std::size_t filter_len_;

  std::deque<double> buffer_;
};

}

#endif
