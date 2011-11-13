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
/*
 * vertex_map.hpp
 *
 *  Created on: Nov 5, 2011
 *      Author: Leo Osvald
 */

#ifndef VERTEX_MAP_HPP_
#define VERTEX_MAP_HPP_

#include <algorithm>
#include <vector>

#include "vertex.hpp"

namespace ksp {

template<typename T>
class VertexMap : public std::vector<T> {
  typedef std::vector<T> Base;
public:
  VertexMap() { }
  VertexMap(std::size_t size) : Base(size) { }
  VertexMap(std::size_t size, const T& val) : Base(size, val) { }
  VertexMap(VertexIdIterator first, VertexIdIterator last, const T& val = T())
    : Base(std::distance(first, last), val) { }

  void Init(VertexIdIterator first, VertexIdIterator last, const T& val = T()) {
    std::fill(Base::begin(), Base::end(), val);
    Base::resize(std::distance(first, last), val);
  }

  typename Base::reference at(VertexId id) throw(NoSuchVertexException) {
    CheckVertexId(id);
    return (*this)[id];
  }

  typename Base::const_reference at(VertexId id) const throw(
      NoSuchVertexException) {
    CheckVertexId(id);
    return (*this)[id];
  }

protected:
  inline void CheckVertexId(VertexId id) const throw(NoSuchVertexException) {
    if (id >= Base::size())
      throw NoSuchVertexException(id);
  }
};

}  // namespace ksp

#endif /* VERTEX_MAP_HPP_ */
