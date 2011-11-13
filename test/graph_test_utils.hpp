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

#ifndef GRAPH_TEST_UTILS_HPP_
#define GRAPH_TEST_UTILS_HPP_

#include <string>
#include <sstream>

#include "../src/graph_builder.hpp"

namespace ksp {

namespace test {

template<typename V, typename T, class VCompare>
Edge<T> E(const GraphBuilder<V, T, VCompare>& g,
          V tail, V head, T edge_data) {
  return Edge<T>(g.vertex_id(tail), g.vertex_id(head), edge_data);
}

template<typename V, typename T, class VCompare>
std::string EStr(const GraphBuilder<V, T, VCompare>& g, const Edge<T>& e) {
  std::ostringstream oss;
  oss << g.vertex(e.tail) << " -> " << g.vertex(e.head) << ": " << e.data;
  return "(" + oss.str() + ")";
}

}  // namespace test

}  // namespace ksp

#endif /* GRAPH_TEST_UTILS_HPP_ */
