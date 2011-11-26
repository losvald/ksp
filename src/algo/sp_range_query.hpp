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

#ifndef SP_RANGE_QUERY_HPP_
#define SP_RANGE_QUERY_HPP_

#include <functional>
#include <vector>

#include "../util/bit_utils.hpp"

#include "../shortest_path_tree.hpp"
#include "../vertex.hpp"
#include "../vertex_map.hpp"

#include "topological_sort.hpp"

namespace ksp {

template<typename T = unsigned, class TClosure = std::plus<T> >
class ShortestPathRangeQuery {
public:
  ShortestPathRangeQuery(const TClosure& closure = TClosure())
  : start_vertex_(kNullVertexId),
    closure_(closure) { }

  ShortestPathRangeQuery(const ShortestPathTree<T>& sp_tree,
                        const TClosure& closure = TClosure())
  : closure_(closure) {
    Init(sp_tree);
  }

  void Init(const ShortestPathTree<T>& sp_tree) {
    std::vector<VertexId> v_toposorted;
    TopologicalSort<T>(sp_tree.edges_begin(), sp_tree.edges_end(),
                       &v_toposorted);
    Init(sp_tree, v_toposorted.begin(), v_toposorted.end());
  }

  template<typename InputIterator>
  void Init(const ShortestPathTree<T>& sp_tree,
            InputIterator vtcs_toposorted_begin,
            InputIterator vtcs_toposorted_end) {
    start_vertex_ = sp_tree.start_vertex();
    sp_range_query_.clear();
    sp_range_query_.resize(sp_tree.distance_map().size());
    skip_.clear();
    skip_.resize(sp_range_query_.size());
    level_.clear();
    level_.resize(sp_range_query_.size());
    for (; vtcs_toposorted_begin != vtcs_toposorted_end;
        ++vtcs_toposorted_begin) {
      VertexId v = *vtcs_toposorted_begin;
      if (!sp_tree.is_reachable(v))
        continue;
      VertexId v_pred = sp_tree.path_rbegin(v)->tail;
      if (v_pred != kNullVertexId) {
        level_[v] = level_[v_pred] + 1;
        std::size_t size = skip_[v_pred].size() +
            (level_[v] == 1U << skip_[v_pred].size());
        skip_[v].resize(size);
        sp_range_query_[v].resize(size);

        skip_[v][0] = v_pred;
        sp_range_query_[v][0] = sp_tree.path_rbegin(v)->data;
        unsigned p_max = skip_[v].size();
        VertexId v_skip_half = skip_[v][0];
        const T* v_closure_half = &sp_range_query_[v][0];
        for (unsigned p = 1, p_half = 0; p < p_max; p_half = p++) {
          v_closure_half = &(sp_range_query_[v][p] = closure_(
              sp_range_query_[v_skip_half][p_half],
              *v_closure_half));
          v_skip_half = skip_[v][p] = skip_[v_skip_half][p_half];
        }
//        for (unsigned p = 0; p < p_max; ++p) {
//          skip_[v][p + 1] = skip_[skip_[v][p]][p];
//          sp_range_query_[v][p + 1] = closure(
//              sp_range_query_[skip_[v][p]][p],
//              sp_range_query_[v][p]);
//        }
      }
    }
  }

  T operator()(VertexId from, VertexId to) const {
    unsigned len = level_[to] - level_[from];
    unsigned chunks_len = SetBitCount(len), chunk_ind = 0;
    T chunks[chunks_len];
    VertexId v = to;
    for (unsigned p = 0; len; ++p, len >>= 1) {
      if (len & 1) {
        chunks[chunk_ind++] = sp_range_query_[v][p];
        v = skip_[v][p];
      }
    }
    T ret = T();
    while (chunks_len--)
      ret = closure_(ret, chunks[chunks_len]);
    return ret;
  }

  inline T operator()(VertexId to) {
    return (*this)(start_vertex_, to);
  }

  inline const TClosure& closure() const {
    return closure_;
  }

  const std::vector<VertexId>& skip_vertices(VertexId v) const {
    return skip_[v];
  }

  VertexId start_vertex() const {
    return start_vertex_;
  }

  bool is_reachable(VertexId v, VertexId from) const {
    if (level_[v] < level_[from])
      return false;
    unsigned len = level_[v] - level_[from];
    for (unsigned p = 0; len; ++p, len >>= 1)
      if (len & 1)
        v = skip_[v][p];
    return v == from;
  }

private:
  VertexId start_vertex_;
  TClosure closure_;
  VertexMap<std::vector<T> > sp_range_query_;
  VertexMap<std::vector<VertexId> > skip_;
  VertexMap<unsigned> level_;
};

}  // namespace ksp

#endif /* SP_RANGE_QUERY_HPP_ */
