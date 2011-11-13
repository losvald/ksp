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

#ifndef SHORTEST_PATH_TREE_HPP_
#define SHORTEST_PATH_TREE_HPP_

#include <iterator>
#include <vector>

#include "vertex.hpp"
#include "vertex_map.hpp"
#include "edge.hpp"

namespace ksp {

template<typename T>
class ShortestPathTreeIterator;

template<typename T>
class ShortestPathTreeEdgeIterator;

template<typename T = unsigned>
class ShortestPathTree {
public:
  typedef Edge<T> EdgeType;
  typedef std::vector<EdgeType> EdgeList;
  typedef ShortestPathTreeIterator<T> Iter;
  typedef ShortestPathTreeEdgeIterator<T> EdgeIter;
  typedef std::vector<VertexId>::const_iterator VertexIter;

  template<class VertexInputIterator>
  ShortestPathTree(VertexInputIterator vertices_begin,
                   VertexInputIterator vertices_end,
                   VertexId start, const T& distance_max) {
    Init(vertices_begin, vertices_end, start, distance_max);
  }

  ShortestPathTree() {
    Init(VertexIdIterator(kNullVertexId), VertexIdIterator(kNullVertexId),
         kNullVertexId, T());
  }

  void Add(const T& new_dist, const EdgeType& backward_edge) {
    reachable_[backward_edge.head] = true;
    vertices_.push_back(backward_edge.head);
    Update(new_dist, backward_edge);
  }

  inline void Update(const T& new_dist, const EdgeType& backward_edge) {
    dist_[backward_edge.head] = new_dist;
    backward_edges_[backward_edge.head] = backward_edge;
  }

  template<class VertexInputIterator>
  void Init(VertexInputIterator vertices_begin,
            VertexInputIterator vertices_end,
            VertexId start, const T& distance_max) {
    const std::size_t vtx_cnt = std::distance(vertices_begin, vertices_end);
    if (vtx_cnt != 0) {
      dist_ = VertexMap<T>(vtx_cnt, distance_max);
      reachable_ = VertexMap<bool>(vtx_cnt);
      vertices_.clear(); vertices_.reserve(vtx_cnt);
      backward_edges_ = EdgeList(vtx_cnt, NullEdge<T>());
      start_ = start;
      dist_[start] = T();
      reachable_[start] = true;
      vertices_.push_back(start);
    }
  }

  inline const T& distance(VertexId v) const throw(NoSuchVertexException) {
    return dist_.at(v);
  }

  const VertexMap<bool>& reachable_map() const {
    return reachable_;
  }

  inline const VertexMap<T>& distance_map() const {
    return dist_;
  }

  VertexId start_vertex() const {
    return start_;
  }

  std::size_t vertex_count() const {
    return vertices_.size();
  }

  VertexIter vertices_begin() const {
    return vertices_.begin();
  }

  VertexIter vertices_end() const {
    return vertices_.end();
  }

  EdgeIter edges_begin() const {
    return EdgeIter(*this, vertices_.begin());
  }

  EdgeIter edges_end() const {
    return EdgeIter(*this, vertices_.end());
  }

  std::size_t edge_count() const {
    return backward_edges_.size() - (backward_edges_.size() == 0);
  }

  Iter path_rbegin(VertexId v) const {
    CheckReachable(v);
    return Iter(*this, v);
  }

  Iter path_rend(VertexId v) const {
    CheckReachable(v);
    return Iter(*this, start_);
  }

  bool is_reachable(VertexId v) const {
    return reachable_.at(v);
  }

private:

  inline void CheckReachable(VertexId v) const throw(VertexUnreachableException,
      NoSuchVertexException) {
    if (!reachable_.at(v))
      throw VertexUnreachableException(v);
  }

  EdgeList backward_edges_;
  VertexId start_;
  VertexMap<T> dist_;
  VertexMap<bool> reachable_;
  std::vector<VertexId> vertices_;

  friend class ShortestPathTreeIterator<T>;
  friend class ShortestPathTreeEdgeIterator<T>;
};


template<typename T = unsigned>
class ShortestPathTreeIterator : public std::iterator<std::forward_iterator_tag,
ptrdiff_t> {
public:
  inline ShortestPathTreeIterator() : tree_(NULL), v_(kNullVertexId) { }

  inline const Edge<T>& operator*() const {
    return tree_->backward_edges_[v_];
  }

  inline const Edge<T>* operator->() const {
    return &tree_->backward_edges_[v_];
  }

  inline ShortestPathTreeIterator& operator++() {
    v_ = tree_->backward_edges_[v_].tail;
    return *this;
  }

  inline ShortestPathTreeIterator operator++(int) {
    ShortestPathTreeIterator old = *this;
    v_ = tree_->backward_edges_[v_].tail;
    return old;
  }

  inline friend bool operator==(const ShortestPathTreeIterator& lhs,
      const ShortestPathTreeIterator& rhs) {
    return lhs.v_ == rhs.v_ && lhs.tree_ == rhs.tree_;
  }

  inline friend bool operator!=(const ShortestPathTreeIterator& lhs,
      const ShortestPathTreeIterator& rhs) {
    return !(lhs == rhs);
  }

private:
  inline ShortestPathTreeIterator(const ShortestPathTree<T>& tree, VertexId v)
  : tree_(&tree), v_(v) { }

  const ShortestPathTree<T>* tree_;
  VertexId v_;

  friend class ShortestPathTree<T>;
};


template<typename T = unsigned>
class ShortestPathTreeEdgeIterator
: public std::iterator<std::bidirectional_iterator_tag, ptrdiff_t> {
public:
  inline const Edge<T>& operator*() const {
    return tree_->backward_edges_[*v_it_];
  }

  inline const Edge<T>* operator->() const {
    return &tree_->backward_edges_[*v_it_];
  }

  inline ShortestPathTreeEdgeIterator& operator++() {
    Increment();
    return *this;
  }

  inline ShortestPathTreeEdgeIterator operator++(int) {
    ShortestPathTreeEdgeIterator old = *this;
    Increment();
    return *this;
  }

  inline ShortestPathTreeEdgeIterator& operator--() {
    Decrement();
    return *this;
  }

  inline ShortestPathTreeEdgeIterator operator--(int) {
    ShortestPathTreeEdgeIterator old = *this;
    Decrement();
    return *this;
  }

  inline friend bool operator==(const ShortestPathTreeEdgeIterator& lhs,
      const ShortestPathTreeEdgeIterator& rhs) {
    return lhs.v_it_ == rhs.v_it_ && lhs.tree_ == rhs.tree_;
  }

  inline friend bool operator!=(const ShortestPathTreeEdgeIterator& lhs,
      const ShortestPathTreeEdgeIterator& rhs) {
    return !(lhs == rhs);
  }

private:

  inline ShortestPathTreeEdgeIterator(
      const ShortestPathTree<T>& tree,
      typename ShortestPathTree<T>::VertexIter v_it)
  : tree_(&tree),
    v_it_(v_it +
          (v_it != tree.vertices_.end() && *v_it == tree.start_vertex())) { }

  inline void Increment() {
    ++v_it_;
    v_it_ += (v_it_ != tree_->vertices_.end() &&
        *v_it_ == tree_->start_vertex());
  }

  inline void Decrement() {
    --v_it_;
    v_it_ -= *v_it_ == tree_->start_vertex();
  }

  const ShortestPathTree<T>* tree_;
  typename ShortestPathTree<T>::VertexIter v_it_;

  friend class ShortestPathTree<T>;
};

}  // namespace ksp

#endif /* SHORTEST_PATH_TREE_HPP_ */
