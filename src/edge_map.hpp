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

#ifndef EDGE_MAP_HPP_
#define EDGE_MAP_HPP_

#include <algorithm>
#include <vector>

#include "edge.hpp"

namespace ksp {

template<typename T = unsigned>
class EdgeMap : public std::vector<T> {
  typedef std::vector<T> Base;
public:
  EdgeMap() { }
  EdgeMap(std::size_t size) : Base(size) { }
  EdgeMap(std::size_t size, const T& val) : Base(size, val) { }

  template<typename InputIterator>
  EdgeMap(InputIterator first, InputIterator last, const T& val = T())
    : Base(std::distance(first, last), val) { }

  template<typename InputIterator>
  void Init(InputIterator first, InputIterator last, const T& val = T()) {
    std::fill(Base::begin(), Base::end(), val);
    Base::resize(std::distance(first, last), val);
  }

  typename Base::reference at(EdgeId id) throw(NoSuchEdgeException) {
    CheckEdgeId(id);
    return (*this)[id];
  }

  typename Base::const_reference at(EdgeId id) const throw(
      NoSuchEdgeException) {
    CheckEdgeId(id);
    return (*this)[id];
  }

protected:
  inline void CheckEdgeId(EdgeId id) const throw(NoSuchEdgeException) {
    if (id >= Base::size())
      throw NoSuchEdgeException(id);
  }
};

}  // namespace ksp

#endif /* EDGE_MAP_HPP_ */
