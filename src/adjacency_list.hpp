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

#ifndef ADJACENCY_LIST_HPP_
#define ADJACENCY_LIST_HPP_

#include <vector>

#include "edge.hpp"
#include "vertex.hpp"

namespace ksp {

static bool gDebugTT; // FIXME

template<typename T = unsigned>
class AdjacencyList {
public:
  typedef typename std::vector<Edge<T> > EdgeList;
  typedef typename EdgeList::iterator Iter;
  typedef typename EdgeList::const_iterator ConstIter;

  template<typename InputIterator>
  AdjacencyList(InputIterator edges_begin, InputIterator edges_end) {
    Init(edges_begin, edges_end);
  }

  inline ConstIter first(VertexId v_id) const {
    return first_.at(v_id);
  }

  inline ConstIter last(VertexId v_id) const {
    const std::size_t next_v_id = v_id + 1;
    if (next_v_id >= first_.size())
      throw NoSuchVertexException(next_v_id);
    return first_[next_v_id];
  }

  inline Iter first(VertexId v_id) {
    return first_[v_id];
  }

  inline Iter last(VertexId v_id) {
    const std::size_t next_v_id = v_id + 1;
    if (next_v_id >= first_.size())
      throw NoSuchVertexException(next_v_id);
    return first_[next_v_id];
  }

  inline std::size_t Count(VertexId v_id) const {
    return last(v_id) - first(v_id);
  }

  friend std::ostream& operator<<(std::ostream& os, const AdjacencyList& a) {
    for (VertexId v = 0; v < a.max_vertex_id(); ++v) {
      os << v << " : ";
      for (typename EdgeList::const_iterator it = a.first(v), last = a.last(v);
          it != last; ++it) {
        os << *it << " ";
      }
      os << std::endl;
    }
    return os;
  }

  std::size_t max_vertex_id() const {
    return first_.size() - 2U;
  }

  std::size_t edge_count() const {
    return adj_.size();
  }

  ConstIter edges_begin() const {
    return adj_.begin();
  }

  ConstIter edges_end() const {
    return adj_.end();
  }

private:

  template<typename InputIterator>
  void Init(InputIterator edges_begin, InputIterator edges_end) {
    if (edges_begin == edges_end) {
      adj_.clear();
      first_.resize(1, adj_.end());
      return;
    }

    std::size_t vid_max = 0, edge_cnt = 0;
    for (InputIterator e = edges_begin; e != edges_end; ++e) {
      vid_max = std::max(vid_max, std::max(e->tail, e->head));
      edge_cnt++;
    }
    adj_.resize(edge_cnt, *edges_begin);
    const Iter adj_begin = adj_.begin();

    // set first_ iterators to positions of last
    first_.resize(vid_max + 1 + 1, adj_begin);
    *(--first_.end()) = adj_.end();  // sentinel for vid_max + 1
    for (InputIterator e = edges_begin; e != edges_end; ++e)
      first_[e->tail]++;
    typename std::vector<Iter>::iterator last_i = first_.begin(),
        last_end = --first_.end();
    for (std::size_t offset = 0; last_i != last_end; ++last_i) {
      offset = (*last_i += offset) - adj_begin;
    }

    // set first_ iterators to correct positions
    for (InputIterator e = edges_begin; e != edges_end; ++e)
      *(--first_[e->tail]) = *e;
  }

  EdgeList adj_;
  std::vector<Iter> first_;
};

}  // namespace ksp

#endif /* ADJACENCY_LIST_HPP_ */
