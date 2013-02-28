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

#include <iostream>
#include <vector>

#include "edge.hpp"
#include "vertex.hpp"

#define FOR_ADJEDGE(type, it, adj, v)                                   \
  for (ksp::AdjacencyList< type >::ConstIter it = (adj).first(v),       \
           last = (adj).last(v); it != last; ++it)

#define FOR_ADJEDGEID(it, adj, v)                                       \
  for (ksp::EdgeIdIterator it = (adj).ids_first(v),                     \
           last = (adj).ids_last(v); it != last; ++it)

#define FOR_ADJLISTEDGES(type, it, adj)                                 \
  for (ksp::AdjacencyList< type >::ConstIter it = (adj).edges_begin(),  \
           last = (adj).edges_end(); it != last; ++it)

namespace ksp {

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

  AdjacencyList(const AdjacencyList& other)
      : adj_(other.adj_),
        first_(other.first_) {
    for (typename std::vector<Iter>::iterator it = first_.begin(),
             it_end = first_.end(); it != it_end; ++it)
      *it += (adj_.begin() - other.adj_.begin());
  }

  AdjacencyList& operator=(AdjacencyList other) {
    Swap(other);
    return *this;
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

  inline std::size_t Count(VertexId v_id) const {
    return last(v_id) - first(v_id);
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

  inline EdgeIdIterator ids_first(VertexId v_id) const {
    return EdgeIdIterator(first(v_id) - adj_.begin());
  }

  inline EdgeIdIterator ids_last(VertexId v_id) const {
    return EdgeIdIterator(last(v_id) - adj_.begin());
  }

  VertexId max_vertex_id() const {
    return first_.size() - 2U;
  }

  EdgeId max_edge_id() const {
    return adj_.size() - 1U;
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

  EdgeIdIterator edge_ids_begin() const {
    return EdgeIdIterator(0U);
  }

  EdgeIdIterator edge_ids_end() const {
    return EdgeIdIterator(max_edge_id() + 1U);
  }

  const Edge<T>& edge(EdgeId id) const {
    return adj_.at(id);
  }

  EdgeId edge_id(ConstIter iter) const {
    return iter - edges_begin();
  }

  EdgeId edge_id(Iter iter) const {
    return iter - edges_begin();
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

  void Swap(AdjacencyList& other) {
    std::swap(adj_, other.adj_);
    std::swap(first_, other.first_);
  }

  EdgeList adj_;
  std::vector<Iter> first_;
};

}  // namespace ksp

#endif /* ADJACENCY_LIST_HPP_ */
