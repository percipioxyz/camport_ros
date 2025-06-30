#include "percipio_camera/percipio_exception.h"
#include <sstream>

namespace percipio_wrapper
{

PercipioException::PercipioException (const std::string& function_name, const std::string& file_name, unsigned line_number, const std::string& message) throw ()
: function_name_ (function_name)
, file_name_ (file_name)
, line_number_ (line_number)
, message_ (message)
{
  std::stringstream sstream;
  sstream << function_name_ << " @ " << file_name_ << " @ " << line_number_ << " : " << message_;
  message_long_ = sstream.str();
}

PercipioException::~PercipioException () throw ()
{
}

PercipioException& PercipioException::operator = (const PercipioException& exception) throw ()
{
  message_ = exception.message_;
  return *this;
}

const char* PercipioException::what () const throw ()
{
  return message_long_.c_str();
}

const std::string& PercipioException::getFunctionName () const throw ()
{
  return function_name_;
}

const std::string& PercipioException::getFileName () const throw ()
{
  return file_name_;
}

unsigned PercipioException::getLineNumber () const throw ()
{
  return line_number_;
}

} //namespace percipio_camera
