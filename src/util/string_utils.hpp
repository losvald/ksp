/*
 * Copyright (C) 2011 Leo Osvald <leo.osvald@gmail.com>
 *
 * This file is part of KSP Library.
 *
 * KSP Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KSP Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STRING_UTILS_HPP_
#define STRING_UTILS_HPP_

#include <string>
#include <sstream>

class MakeString {
public:
  operator std::string() const;
  operator const char*() const;

  template<typename T>
  MakeString& operator<<(T const & datum) {
    buffer_ << datum;
    return *this;
  }

private:
   std::ostringstream buffer_;
};

std::string ToLowerCase(const std::string& s);

bool EqualsIgnoreCase(const std::string& a, const std::string& b);

template<class InputIterator>
std::string ToString(InputIterator first, InputIterator last) {
  std::ostringstream oss;
  oss << "[";
  if (first != last) {
    while (true) {
      oss << *first;
      if (++first == last)
        break;
      oss << ", ";
    }
  }
  oss << "]";
  return oss.str();
}

template<class InputIterator>
std::string MapToString(InputIterator first, InputIterator last) {
  std::ostringstream oss;
  oss << "{";
  if (first != last) {
    while (true) {
      oss << first->first << " => " << first->second;
      if (++first == last)
        break;
      oss << ", ";
    }
  }
  oss << "}";
  return oss.str();
}

#endif /* STRING_UTILS_HPP_ */
