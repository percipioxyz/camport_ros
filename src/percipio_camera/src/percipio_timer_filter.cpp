#include "percipio_camera/percipio_timer_filter.h"
#include <algorithm>


namespace percipio_wrapper
{

PercipioTimerFilter::PercipioTimerFilter(std::size_t filter_len):
    filter_len_(filter_len)
{
}

PercipioTimerFilter::~PercipioTimerFilter()
{
}

void PercipioTimerFilter::addSample(double sample)
{
  buffer_.push_back(sample);
  if (buffer_.size()>filter_len_)
    buffer_.pop_front();
}

double PercipioTimerFilter::getMedian()
{
  if (buffer_.size()>0)
  {
    std::deque<double> sort_buffer = buffer_;

    std::sort(sort_buffer.begin(), sort_buffer.end());

    return sort_buffer[sort_buffer.size()/2];
  } else
    return 0.0;
}

double PercipioTimerFilter::getMovingAvg()
{
  if (buffer_.size() > 0)
  {
    double sum = 0;

    std::deque<double>::const_iterator it = buffer_.begin();
    std::deque<double>::const_iterator it_end = buffer_.end();

    while (it != it_end)
    {
      sum += *(it++);
    }

    return sum / static_cast<double>(buffer_.size());
  } else
    return 0.0;
}


void PercipioTimerFilter::clear()
{
  buffer_.clear();
}


} //namespace percipio_wrapper
