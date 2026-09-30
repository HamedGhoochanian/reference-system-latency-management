#ifndef REFERENCE_SYSTEM__STDOUT_MUTEX_HPP_
#define REFERENCE_SYSTEM__STDOUT_MUTEX_HPP_

#include <iostream>
#include <mutex>
#include <string>

inline std::mutex & reference_system_cout_mutex()
{
  static std::mutex mutex;
  return mutex;
}

inline void reference_system_write_stdout_line(const std::string & line)
{
  std::lock_guard<std::mutex> lock(reference_system_cout_mutex());
  std::cout << line << std::endl;
}

#endif  // REFERENCE_SYSTEM__STDOUT_MUTEX_HPP_
