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

#ifndef DIJKSTRA_MM_HPP_
#define DIJKSTRA_MM_HPP_

#include <cassert>
#include <limits>
#include <map>

#include "../../src/adjacency_list.hpp"
#include "../../src/shortest_path_tree.hpp"
#include "../../src/vertex_map.hpp"

namespace ksp {

template<typename T, class TCompare, class TClosure, typename InputIterator,
class TDijkstraVisitor, class VisitorContext>
void DijkstraShortestPathsMM(InputIterator edges_begin, InputIterator edges_end,
                             VertexId start_id,
                             ShortestPathTree<T>* const sp_tree,
                             const T& distance_max,
                             const TCompare& compare,
                             const TClosure& closure,
                             const TDijkstraVisitor& visitor,
                             VisitorContext* visitor_context) {
  AdjacencyList<T> adj(edges_begin, edges_end);
  const unsigned vertex_count = adj.max_vertex_id() + 1;
  VertexIdIterator vertices_begin = VertexIdIterator(),
      vertices_end = VertexIdIterator(vertex_count);
  VertexMap<bool> visited(vertex_count);

  sp_tree->Init(vertices_begin, vertices_end, start_id, distance_max);
  const VertexMap<T>& d = sp_tree->distance_map();
  const VertexMap<bool>& reachable = sp_tree->reachable_map();
  for (VertexIdIterator v_id = vertices_begin, v_id_to = vertices_end;
      v_id != v_id_to; ++v_id)
    visitor.InitVertex(*v_id, *sp_tree, visitor_context);

  typedef std::multimap<T, VertexId, TCompare> MinHeap;
  MinHeap dist;
  const typename MinHeap::iterator dist_end = dist.end();
  VertexMap<typename MinHeap::iterator> its(vertex_count, dist.end());
  its[start_id] = dist.insert(std::make_pair(T(), start_id));
  while (!dist.empty()) {
    // extract min dist vertex and its distance
    typename MinHeap::const_iterator min_dist_it = dist.begin();
    assert(dist.size() <= vertex_count);
    VertexId u_id = min_dist_it->second;
    T u_dist = min_dist_it->first;
    dist.erase(dist.begin());
    its[u_id] = dist.end();
    if (visited[u_id])
      continue;
    visited[u_id] = true;
    visitor.ExamineVertex(u_id, *sp_tree, visitor_context);

    // update distance to adjacent vertices
    for (typename AdjacencyList<T>::ConstIter e = adj.first(u_id),
        e_last = adj.last(u_id); e != e_last; ++e) {
      visitor.ExamineEdge(*e, *sp_tree, visitor_context);
      VertexId v_id = e->head;
      if (visited[v_id]) {  // black - cannot relax since distance already min
        visitor.EdgeNotRelaxed(*e, *sp_tree, visitor_context);
      } else {
        const T& v_dist_new = closure(u_dist, e->data);
        if (reachable[v_id]) {  // gray - try to relax
          const T& v_dist_old = d[v_id];
          visitor.ExamineVertex(v_id, *sp_tree, visitor_context);
          if (compare(v_dist_new, v_dist_old)) {
            sp_tree->Update(v_dist_new, *e);
            const typename MinHeap::iterator& it_old = its[v_id];
            if (it_old != dist_end)
              dist.erase(it_old);
            its[v_id] = dist.insert(std::make_pair(v_dist_new, v_id));
            visitor.EdgeRelaxed(*e, *sp_tree, visitor_context);
          } else
            visitor.EdgeNotRelaxed(*e, *sp_tree, visitor_context);
        } else {  // white - newly discovered vertex
          sp_tree->Add(v_dist_new, *e);
          its[v_id] = dist.insert(std::make_pair(v_dist_new, v_id));
          visitor.DiscoverVertex(v_id, *sp_tree, visitor_context);
          visitor.EdgeRelaxed(*e, *sp_tree, visitor_context);
        }
      }
    }
    visitor.FinishVertex(u_id, *sp_tree, visitor_context);
  }
}

template<typename T, class TCompare, class TClosure, typename InputIterator>
void DijkstraShortestPathsMM(InputIterator edges_begin, InputIterator edges_end,
                             VertexId start_id,
                             ShortestPathTree<T>* const sp_tree,
                             const T& distance_max,
                             const TCompare& compare, const TClosure& closure) {
  typedef DijkstraVisitorAdapter<T, void> NullDijkstraVisitorType;
  DijkstraShortestPathsMM<T, TCompare, TClosure, InputIterator,
  NullDijkstraVisitorType, typename NullDijkstraVisitorType::ContextType>(
      edges_begin, edges_end, start_id, sp_tree,
      distance_max, compare, closure,
      NullDijkstraVisitorType(), NULL);
}

template<typename T, typename InputIterator>
void DijkstraShortestPathsMM(InputIterator edges_begin, InputIterator edges_end,
                           VertexId start_id,
                           ShortestPathTree<T>* const sp_tree) {
  DijkstraShortestPathsMM(edges_begin, edges_end, start_id, sp_tree,
                          std::numeric_limits<T>::max(),
                          std::less<T>(), std::plus<T>());
}


}  // namespace ksp

#endif /* DIJKSTRA_MM_HPP_ */
