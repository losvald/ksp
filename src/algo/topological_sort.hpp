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

#ifndef TOPOLOGICAL_SORT_HPP_
#define TOPOLOGICAL_SORT_HPP_

#include <stack>

#include "../adjacency_list.hpp"
#include "../vertex.hpp"
#include "../vertex_map.hpp"

namespace ksp {

template<typename T>
bool TopologicalSort(const AdjacencyList<T>& adj, std::vector<VertexId>* l) {
  const VertexIdIterator vertices_begin = VertexIdIterator(),
      vertices_end = VertexIdIterator(adj.max_vertex_id() + 1);
  std::stack<VertexId> s;
  VertexMap<unsigned> deg_in(vertices_begin, vertices_end);
  for (typename AdjacencyList<T>::ConstIter e = adj.edges_begin(),
      e_end = adj.edges_end(); e != e_end; ++e) {
    ++deg_in[e->head];
  }
  for (VertexIdIterator v = vertices_begin; v != vertices_end; ++v) {
    if (!deg_in[*v])
      s.push(*v);
  }
  l->clear();
  l->reserve(deg_in.size());
  while (!s.empty()) {
    VertexId u = s.top(); s.pop();
    l->push_back(u);
    for (typename AdjacencyList<T>::ConstIter e = adj.first(u),
        e_end = adj.last(u); e != e_end; ++e)
      if (!--deg_in[e->head])
        s.push(e->head);
  }
  return s.empty();
}

template<typename T, typename InputIterator>
bool TopologicalSort(InputIterator edges_begin, InputIterator edges_end,
                     std::vector<VertexId>* l) {
  return TopologicalSort(AdjacencyList<T>(edges_begin, edges_end), l);
}



}  // namespace ksp

#endif /* TOPOLOGICAL_SORT_HPP_ */
