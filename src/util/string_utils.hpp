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

#include <map>
#include <set>
#include <string>
#include <sstream>
#include <vector>

class MakeString {
public:
  operator std::string() const;
  operator const char*() const;

  template<typename T>
  MakeString& operator<<(T const & datum) {
    buffer_ << ToString(datum);
    return *this;
  }

  MakeString& operator<<(const char* datum) {
    buffer_ << (datum != NULL ? datum : "(null)");
    return *this;
  }

private:
   std::ostringstream buffer_;
};

std::string ToLowerCase(const std::string& s);

bool EqualsIgnoreCase(const std::string& a, const std::string& b);

template<typename T>
inline std::string ToString(const T& t) {
  std::ostringstream oss;
  oss << t;
  return oss.str();
}

inline std::string ToString(const char* s) {
  if (s == NULL)
    return "(null)";
  return s;
}

template<class InputIterator>
std::string ToString(InputIterator first, InputIterator last, const char* sep,
                     const char* begin, const char* end) {
  std::ostringstream oss;
  oss << begin;
  if (first != last) {
    while (true) {
      oss << ToString(*first);
      if (++first == last)
        break;
      oss << sep;
    }
  }
  oss << end;
  return oss.str();
}

template<class InputIterator>
inline std::string ToString(InputIterator first, InputIterator last) {
  return ToString(first, last, ", ", "[", "]");
}

template<typename T>
inline std::string ToString(const std::vector<T>& v) {
  return ToString(v.begin(), v.end());
}

template<typename T>
inline std::string ToString(const std::set<T>& s) {
  return ToString(s.begin(), s.end(), ", ", "{", "}");
}

template<typename T>
inline std::string ToString(const std::multiset<T>& s) {
  return ToString(s.begin(), s.end(), ", ", "{", "}");
}

template<class InputIterator>
std::string MapToString(InputIterator first, InputIterator last) {
  std::ostringstream oss;
  oss << "{";
  if (first != last) {
    while (true) {
      oss << ToString(first->first) << " => " << ToString(first->second);
      if (++first == last)
        break;
      oss << ", ";
    }
  }
  oss << "}";
  return oss.str();
}

template<typename K, typename V>
inline std::string ToString(const std::map<K, V>& m) {
  return MapToString(m.begin(), m.end());
}

template<typename K, typename V>
inline std::string ToString(const std::multimap<K, V>& m) {
  return MapToString(m.begin(), m.end());
}

#endif /* STRING_UTILS_HPP_ */
