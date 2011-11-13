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

#ifndef GRAPH_IO_HPP_
#define GRAPH_IO_HPP_

#include <iostream>

#include "../graph_builder.hpp"

namespace ksp {

template<typename V, typename T, class VCompare>
void ReadGraph(std::istream& is,
               GraphBuilder<V, T, VCompare>* g) {
  g->Clear();
  std::size_t n, m;
  is >> n >> m;
  for (std::size_t i = 0; i < n; ++i) {
    V v;
    is >> v;
    g->AddVertex(v);
  }
  for (std::size_t i = m; i != 0; --i) {
    V tail, head;
    T edge_data;
    is >> tail >> head >> edge_data;
    g->AddEdge(tail, head, edge_data);
  }
}

}  // namespace ksp

#endif /* GRAPH_IO_HPP_ */
