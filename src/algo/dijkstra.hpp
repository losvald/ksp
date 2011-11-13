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

#ifndef DIJKSTRA_HPP_
#define DIJKSTRA_HPP_

#include <algorithm>
#include <queue>
#include <limits>

#include "../adjacency_list.hpp"
#include "../shortest_path_tree.hpp"
#include "../vertex_map.hpp"

namespace ksp {

namespace {

template<typename T, class TCompare>
class MinDistanceIntervalTree {
public:
  MinDistanceIntervalTree(const VertexMap<T>& distance_map,
                          const T& distance_max,
                          const TCompare& compare)
  : d_(distance_map),
    popped_(d_.size()),
    size_(d_.size()),
    offset_(ComputeOffset(size_)),
    d_min_(new T[offset_]),
    d_max_(distance_max),
    compare_(compare) {
    std::fill(d_min_, d_min_ + offset_, distance_max);
  }

  ~MinDistanceIntervalTree() {
    delete d_min_;
  }

  void Update(VertexId n) {
    const T& d_n = d_[n];
    std::size_t n_sib = n ^ 1;
    T d_sib = (n_sib < d_.size() && !popped_[n_sib] ? d_[n_sib] : d_max_);

    if (compare_(d_n, d_sib))
      for (std::size_t i = (n + offset_)>>1; compare_(d_n, d_min_[i]); i >>= 1)
        d_min_[i] = d_n;
  }

  VertexId PopMin() {
    if (empty())
      throw NoSuchVertexException();
    --size_;

    std::size_t n_min = 1;
    while ((n_min <<= 1) < offset_)
      n_min |= compare_(d_min_[n_min | 1], d_min_[n_min]);
    n_min -= offset_;

    // make n_min be minimum unpopped among n and (n + 1)
    // and n_sib the other one
    std::size_t n_sib = n_min | 1;
    n_min ^= (popped_[n_min] || (n_sib < d_.size() && !popped_[n_sib] &&
        compare_(d_[n_sib], d_[n_min])));
    n_sib = n_min ^ 1;
    T d_min = d_[n_min];
    T d_sib = (n_sib < d_.size() && !popped_[n_sib] ? d_[n_sib] : d_max_);

    // update the tree by replacing the necessary nodes with 2nd min
    if (compare_(d_min, d_sib)) {
      T d_min2 = d_sib;
      for (std::size_t n = n_min + offset_; ;) {
        d_min_[n >>= 1] = d_min2;
        if (n == 1 || !compare_(d_min, d_sib = d_min_[n ^ 1]))
          break;
        if (compare_(d_sib, d_min2))
          d_min2 = d_sib;
      }
    }
    popped_[n_min] = true;
    return n_min;
  }

  inline bool Contains(VertexId v) const {
    return !popped_[v];
  }

  inline bool empty() const {
    return size_ == 0;
  }

private:
  static std::size_t ComputeOffset(std::size_t size) {
    std::size_t offset = 1;
    while (offset < size)
      offset <<= 1;
    return offset;
  }

  const VertexMap<T>& d_;
  VertexMap<bool> popped_;

  std::size_t size_;
  std::size_t offset_;
  T* d_min_;

  T d_max_;
  TCompare compare_;
};

}  // namespace


template<typename T = unsigned, class Context = void>
class DijkstraVisitorAdapter {
public:
  typedef Context ContextType;

  void InitVertex(VertexId v_id, const ShortestPathTree<T>& spt,
                  Context* context) const {
  }

  void ExamineVertex(VertexId v_id, const ShortestPathTree<T>& spt,
                     Context* context) const {
  }

  void ExamineEdge(const Edge<T>& e, const ShortestPathTree<T>& spt,
                   Context* context) const {
  }

  void DiscoverVertex(VertexId v_id, const ShortestPathTree<T>& spt,
                      Context* context) const {
  }

  void EdgeRelaxed(const Edge<T>& e, const ShortestPathTree<T>& spt,
                   Context* context) const {
  }

  void EdgeNotRelaxed(const Edge<T>& e, const ShortestPathTree<T>& spt,
                      Context* context) const {
  }

  void FinishVertex(VertexId v_id, const ShortestPathTree<T>& spt,
                    Context* context) const {
  }
};


template<typename T, class TCompare, class TClosure, typename InputIterator,
class TDijkstraVisitor, class VisitorContext>
void DijkstraShortestPaths(InputIterator edges_begin, InputIterator edges_end,
                           VertexId start_id,
                           ShortestPathTree<T>* const sp_tree,
                           const T& distance_max,
                           const TCompare& compare,
                           const TClosure& closure,
                           const TDijkstraVisitor& visitor,
                           VisitorContext* visitor_context) {

  AdjacencyList<T> adj(edges_begin, edges_end);
  VertexIdIterator vertices_begin = VertexIdIterator(),
      vertices_end = VertexIdIterator(adj.max_vertex_id() + 1);

  sp_tree->Init(vertices_begin, vertices_end, start_id, distance_max);
  const VertexMap<T>& d = sp_tree->distance_map();
  const VertexMap<bool>& reachable = sp_tree->reachable_map();
  for (VertexIdIterator v_id = vertices_begin, v_id_to = vertices_end;
      v_id != v_id_to; ++v_id)
    visitor.InitVertex(*v_id, *sp_tree, visitor_context);

  MinDistanceIntervalTree<T, TCompare> interval_tree(d, distance_max, compare);
  interval_tree.Update(start_id);
  while (!interval_tree.empty()) {
    // extract min dist vertex and its distance
    VertexId u_id = interval_tree.PopMin();
    if (!sp_tree->is_reachable(u_id))
      continue;
    T u_dist = sp_tree->distance(u_id);
    visitor.ExamineVertex(u_id, *sp_tree, visitor_context);

    // update distance to adjacent vertices
    for (typename AdjacencyList<T>::ConstIter e = adj.first(u_id),
        e_last = adj.last(u_id); e != e_last; ++e) {
      visitor.ExamineEdge(*e, *sp_tree, visitor_context);
      VertexId v_id = e->head;
      if (!interval_tree.Contains(v_id)) {  // black - cannot relax (distance is min)
        visitor.EdgeNotRelaxed(*e, *sp_tree, visitor_context);
      } else {
        T v_dist_new = closure(u_dist, e->data);
        if (reachable[v_id]) {  // gray - try to relax
          const T& v_dist_old = d[v_id];
          visitor.ExamineVertex(v_id, *sp_tree, visitor_context);
          if (compare(v_dist_new, v_dist_old)) {
            sp_tree->Update(v_dist_new, *e);
            interval_tree.Update(v_id);
            visitor.EdgeRelaxed(*e, *sp_tree, visitor_context);
          } else
            visitor.EdgeNotRelaxed(*e, *sp_tree, visitor_context);
        } else {  // white - newly discovered vertex
          sp_tree->Add(v_dist_new, *e);
          interval_tree.Update(v_id);
          visitor.DiscoverVertex(v_id, *sp_tree, visitor_context);
          visitor.EdgeRelaxed(*e, *sp_tree, visitor_context);
        }
      }
    }
    visitor.FinishVertex(u_id, *sp_tree, visitor_context);
  }
}

template<typename T, class TCompare, class TClosure, typename InputIterator>
void DijkstraShortestPaths(InputIterator edges_begin, InputIterator edges_end,
                           VertexId start_id,
                           ShortestPathTree<T>* const sp_tree,
                           const T& max_distance,
                           const TCompare& compare, const TClosure& closure) {
  typedef DijkstraVisitorAdapter<T, void> NullDijkstraVisitorType;
  DijkstraShortestPaths<T, TCompare, TClosure, InputIterator,
  NullDijkstraVisitorType, typename NullDijkstraVisitorType::ContextType>(
      edges_begin, edges_end, start_id, sp_tree, max_distance,
      compare, closure,
      NullDijkstraVisitorType(), NULL);
}

template<typename T, typename InputIterator>
void DijkstraShortestPaths(InputIterator edges_begin, InputIterator edges_end,
                           VertexId start_id,
                           ShortestPathTree<T>* const sp_tree) {
  DijkstraShortestPaths(edges_begin, edges_end, start_id, sp_tree,
                        std::numeric_limits<T>::max(),
                        std::less<T>(), std::plus<T>());
}


}  // namespace ksp

#endif /* DIJKSTRA_HPP_ */
