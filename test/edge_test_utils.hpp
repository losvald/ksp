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

#ifndef EDGE_TEST_UTILS_HPP_
#define EDGE_TEST_UTILS_HPP_

#include <sstream>
#include <string>

#include "../src/edge.hpp"

namespace ksp {

namespace test {

template<typename T = unsigned>
struct EdgeLess {
  bool operator()(const Edge<T>& x, const Edge<T>& y) const {
      return x.tail < y.tail || (x.tail == y.tail &&
          (x.head < y.head || (x.head == y.head &&
              x.data < y.data)));
  }
};

template<typename T = unsigned>
struct EdgeEqual {
  bool operator()(const Edge<T>& x, const Edge<T>& y) const {
    return x.tail == y.tail && x.head == y.head &&
        x.data == y.data;
  }
};


template<typename T>
std::string EToStr(const Edge<T>& e) {
  std::ostringstream oss;
  oss << "(" << e.tail << " -> " << e.head << ": " << e.data << ")";
  return oss.str();
}

}  // namespace test

}  // namespace ksp

#endif /* EDGE_TEST_UTILS_HPP_ */
