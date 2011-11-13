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

#ifndef EDGE_HPP_
#define EDGE_HPP_

#include <algorithm>
#include <iostream>

#include "vertex.hpp"

namespace ksp {

template<typename T = unsigned>
class Edge {
public:
  VertexId tail;
  VertexId head;
  T data;

  inline Edge(const T& tail, const T& head, const T& data)
  : tail(tail), head(head), data(data) { }

  friend std::ostream& operator<<(std::ostream& os, const Edge& e) {
    os << "(" << e.tail << " -> " << e.head << ": " << e.data << ")";
    return os;
  }

  inline void Reverse() {
    std::swap(tail, head);
  }
};

template<typename T>
Edge<T> NullEdge() {
  static Edge<T> kNullEdge(kNullVertexId, kNullVertexId, T());
  return kNullEdge;
}

}  // namespace ksp

#endif /* EDGE_HPP_ */
