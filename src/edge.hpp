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
#include <exception>
#include <iostream>

#include "vertex.hpp"

#include "util/counter_iterator.hpp"

namespace ksp {

typedef std::size_t EdgeId;
typedef CounterIterator<EdgeId> EdgeIdIterator;

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


const EdgeId kNullEdgeId = -1;

struct EdgeException : public std::exception {
  EdgeException() : e_(NULL) { }
  EdgeException(EdgeId e) : e_(new EdgeId(e)) { }
  virtual ~EdgeException() throw() {
    delete e_;
  }
protected:
  EdgeId* e_;
};

struct NoSuchEdgeException : public EdgeException {
  NoSuchEdgeException();
  NoSuchEdgeException(EdgeId e);
  virtual const char* what() const throw();
};

}  // namespace ksp

#endif /* EDGE_HPP_ */
