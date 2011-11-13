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

#ifndef SPT_TEST_UTILS_HPP_
#define SPT_TEST_UTILS_HPP_

#include <functional>
#include <stack>
#include <vector>

#include "../src/adjacency_list.hpp"
#include "../src/shortest_path_tree.hpp"
#include "../src/vertex_map.hpp"

#include <set>

namespace ksp {

namespace test {

namespace {

  template<typename T>
  bool IsTree(const ShortestPathTree<T>& spt) {
    if (spt.edge_count() + 1 != spt.vertex_count())
      return false;

    try {
      typedef typename AdjacencyList<T>::Iter AdjIter;
      AdjacencyList<T> adj(spt.edges_begin(), spt.edges_end());
      VertexMap<bool> visited(spt.vertex_count());
      std::stack<VertexId> s;
      for (s.push(spt.start_vertex()); s.empty(); ) {
        VertexId u = s.top(); s.pop();
        for (AdjIter e = adj.First(u); e != adj.Last(u); ++e) {
          VertexId v = e->head();
          if (!visited.at(v)) {
            visited[v] = true;
            s.push(v);
          }
        }
      }
      return visited.size() == spt.vertex_count();
    } catch(const VertexException& e) {
      return false;
    }
  }

}  // namespace

template<typename T, class TCompare, class TClosure>
bool IsValidSpt(const ShortestPathTree<T>& spt,
                const TCompare& compare = std::less<T>(),
                const TClosure& closure = std::plus<T>()) {
  const T zero = T();
  if (spt.distance(spt.start_vertex()) != zero)
    return false;

  for (typename ShortestPathTree<T>::EdgeIter e = spt.edges_begin(),
      e_end = spt.edges_end(); e != e_end; ++e) {
    T d = closure(e->data, spt.distance(e->tail)), d2 = spt.distance(e->head);
    if (compare(d, d2) || compare(d2, d))
      return false;
  }
  return true;
}

template<typename T>
std::vector<VertexId> SortedVertices(const ShortestPathTree<T>& spt) {
  std::vector<VertexId> vtcs(spt.vertices_begin(), spt.vertices_end());
  std::sort(vtcs.begin(), vtcs.end());
  return vtcs;
}

template<typename T, class EEdgeEqual>
bool ShortestPathEqual(const ShortestPathTree<T>& lhs_spt, VertexId lhs_v,
                       const ShortestPathTree<T>& rhs_spt, VertexId rhs_v,
                       const EEdgeEqual& edge_eq = EdgeEqual<T>()) {
  if (lhs_v != rhs_v)
    return false;
  ShortestPathTreeIterator<T>
    lhs_path_rend = lhs_spt.path_rbegin(lhs_v),
    rhs_path_rend = lhs_spt.path_rbegin(rhs_v),
    lhs_path_rit = lhs_spt.path_rbegin(lhs_v),
    rhs_path_rit = lhs_spt.path_rbegin(rhs_v);
  for (; lhs_path_rit != lhs_path_rend && rhs_path_rit != rhs_path_rend;
      ++lhs_path_rit, ++rhs_path_rit) {
    if (!edge_eq(*lhs_path_rit, *rhs_path_rit))
      return false;
  }
  return lhs_path_rit == lhs_path_rend && rhs_path_rit == rhs_path_rend;
}

//template<typename T, class TCompare, class TClosure>
//bool ShortestPathEquivalent(const ShortestPathTree<T>& lhs_spt, VertexId lhs_v,
//                            const ShortestPathTree<T>& rhs_spt, VertexId rhs_v,
//                            const TCompare& edge_compare = std::less<T>(),
//                            const TClosure& closure = std::plus<T>()) {
//
//  ShortestPathTreeIterator<T>
//    lhs_path_rend = lhs_spt.path_rbegin(lhs_v),
//    rhs_path_rend = lhs_spt.path_rbegin(rhs_v),
//    lhs_path_rit = lhs_spt.path_rbegin(lhs_v),
//    rhs_path_rit = lhs_spt.path_rbegin(rhs_v);
//}

template<typename T = unsigned, class EEdgeEqual = EdgeEqual<T> >
class SptEqual {
public:
  bool operator()(const ShortestPathTree<T>& lhs,
                  const ShortestPathTree<T>& rhs) const {
    if (SortedVertices(lhs) != SortedVertices(rhs))
      return false;

    for (typename ShortestPathTree<T>::VertexIter v_id = lhs.vertices_begin();
        v_id != lhs.vertices_end(); ++v_id) {
      if (!ShortestPathEqual(lhs, *v_id, rhs, *v_id, EEdgeEqual()))
        return false;
    }
    return true;
  }
};

template<typename T = unsigned, class TCompare = std::less<T>,
    class TClosure = std::plus<T> >
class SptEquiv {
public:
  bool operator()(const ShortestPathTree<T>& lhs,
                  const ShortestPathTree<T>& rhs) const {
    std::vector<VertexId>
        lhs_svtcs = SortedVertices(lhs),
        rhs_svtcs = SortedVertices(rhs);
    if (lhs_svtcs != rhs_svtcs)
      return false;
    TCompare compare;
    TClosure closure;
    for (std::size_t i = 0; i < lhs_svtcs.size(); ++i) {
      const T& lhs_dist = lhs.distance(lhs_svtcs[i]);
      const T& rhs_dist = rhs.distance(rhs_svtcs[i]);
      if (compare(lhs_dist, rhs_dist) || compare(rhs_dist, lhs_dist))
        return false;
    }
    return IsValidSpt(lhs, compare, closure) &&
        IsValidSpt(rhs, compare, closure);
  }
};

}  // namespace test

}  // namespace ksp

#endif /* SPT_TEST_UTILS_HPP_ */
