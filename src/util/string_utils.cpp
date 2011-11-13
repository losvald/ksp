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

#include <cctype>

#include <algorithm>
#include <sstream>

#include "string_utils.hpp"

MakeString::operator std::string() const {
  return buffer_.str();
}

MakeString::operator const char*() const {
  return buffer_.str().c_str();
}

std::string ToLowerCase(const std::string& s) {
  std::string ret(s);
  std::transform(ret.begin(), ret.end(), ret.begin(), ::tolower);
  return ret;
}

bool EqualsIgnoreCase(const std::string& a, const std::string& b) {
  return ToLowerCase(a) == ToLowerCase(b);
}
