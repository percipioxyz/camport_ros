/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 14:00:15
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-04 14:03:24
 */

#ifndef __PERCIPIO_EXCEPTION__
#define __PERCIPIO_EXCEPTION__

#include <cstdarg>
#include <cstdio>
#include <exception>
#include <string>

#if defined _WIN32 && defined _MSC_VER
# define __PRETTY_FUNCTION__ __FUNCTION__
#endif
#define THROW_PERCIPIO_EXCEPTION(format,...) throwPercipioException( __PRETTY_FUNCTION__, __FILE__, __LINE__, format , ##__VA_ARGS__ )

namespace percipio_wrapper
{
/**
 * @brief General exception class
 * @author Suat Gedikli
 * @date 02.january 2011
 */
class PercipioException : public std::exception
{
public:
  PercipioException(const std::string& function_name,
                   const std::string& file_name,
                   unsigned line_number,
                   const std::string& message) throw ();

  virtual ~PercipioException() throw ();
  PercipioException & operator=(const PercipioException& exception) throw ();
  virtual const char* what() const throw ();

  const std::string& getFunctionName() const throw ();
  const std::string& getFileName() const throw ();
  unsigned getLineNumber() const throw ();

protected:
  std::string function_name_;
  std::string file_name_;
  unsigned line_number_;
  std::string message_;
  std::string message_long_;
};

inline void throwPercipioException(const char* function, const char* file, unsigned line, const char* format, ...)
{
  static char msg[1024];
  va_list args;
  va_start(args, format);
  vsprintf(msg, format, args);
  throw PercipioException(function, file, line, msg);
}
} // namespace percipio_camera
#endif
