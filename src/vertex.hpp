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

#ifndef VERTEX_HPP_
#define VERTEX_HPP_

#include <cstddef>
#include <exception>

#include "util/counter_iterator.hpp"

namespace ksp {

typedef std::size_t VertexId;
typedef CounterIterator<VertexId> VertexIdIterator;

const VertexId kNullVertexId = -1;

struct VertexException : public std::exception {
  VertexException() : v_(NULL) { }
  VertexException(VertexId v) : v_(new VertexId(v)) { }
  virtual ~VertexException() throw() {
    delete v_;
  }
protected:
  VertexId* v_;
};

struct NoSuchVertexException : public VertexException {
  NoSuchVertexException();
  NoSuchVertexException(VertexId v);
  virtual const char* what() const throw();
};

struct VertexUnreachableException : public VertexException {
  VertexUnreachableException(VertexId v);
  virtual const char* what() const throw();
};


}  // namespace ksp

#endif /* VERTEX_HPP_ */
