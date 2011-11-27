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

#ifndef EPPSTEIN_PATH_TREE_HPP_
#define EPPSTEIN_PATH_TREE_HPP_

#include <cassert>

#include <functional>
#include <stack>
#include <vector>

#include "../../edge.hpp"
#include "../../shortest_path_tree.hpp"
#include "../../vertex.hpp"

#include "../sp_range_query.hpp"

namespace ksp {

template<typename T, class TClosure>
class EppsteinPathTreeIterator;

template<typename T, class TClosure>
class PathIterator;

template<typename T = unsigned, class TClosure = std::plus<T> >
class EppsteinPathTree {
public:
  typedef EppsteinPathTreeIterator<T, TClosure> SidetrackIterator;
  typedef PathIterator<T, TClosure> Iterator;

  EppsteinPathTree() : sp_tree_(NULL), sp_range_query_(NULL) { }

  EppsteinPathTree(
      const ShortestPathRangeQuery<T, TClosure>* sp_range_query,
      const ShortestPathTree<T>* sp_tree, VertexId start_vertex) {
    Init(sp_range_query, sp_tree, start_vertex);
  }

  void Init(const ShortestPathRangeQuery<T, TClosure>* sp_range_query,
            const ShortestPathTree<T>* sp_tree, VertexId start_vertex) {
    sp_tree_ = sp_tree;
    sp_range_query_ = sp_range_query;
    start_vertex_ = start_vertex;
    paths_.clear();
    paths_.resize(1, Node<T>(
        Edge<T>(start_vertex_, start_vertex_, T()),
        0));
    d_.clear();
    d_.resize(1);
  }

  std::size_t AddPath(const Edge<T>& last_sidetrack,
                      std::size_t prefpath_ord) {
    std::size_t path_ord = paths_.size();
    paths_.push_back(Node<T>(last_sidetrack, prefpath_ord));
    const Node<T>& prefpath = paths_[prefpath_ord];
    assert(sp_range_query_->is_reachable(prefpath.last_sidetrack.head,
                                         last_sidetrack.tail));
    if (prefpath.last_sidetrack.head != last_sidetrack.tail) {
      const T& d_gap = (*sp_range_query_)(last_sidetrack.tail,
          prefpath.last_sidetrack.head);
      d_.push_back(closure()(d_[prefpath_ord],
          closure()(d_gap, last_sidetrack.data)));
    } else {
      d_.push_back(closure()(d_[prefpath_ord], last_sidetrack.data));
    }
    return path_ord;
  }

  void ClearPaths() {
    Init(sp_range_query_, sp_tree_, start_vertex_);
  }

  VertexId start_vertex() const {
    return start_vertex_;
  }

  VertexId end_vertex() const {
    return sp_range_query_->start_vertex();
  }

  std::size_t k() const {
    return paths_.size();
  }

  T distance(std::size_t path_ord) const {
    const T& d = sp_tree_->distance(paths_[path_ord].last_sidetrack.head);
    return path_ord ? closure()(d_[path_ord], d) : d;
  }

  std::size_t prefpath_ord(std::size_t path_ord) const {
    return paths_[path_ord].prefpath_ord;
  }

  SidetrackIterator sidetracks_rbegin(std::size_t path_ord) const {
    return SidetrackIterator(this, path_ord);
  }

  SidetrackIterator sidetracks_rend() const {
    return SidetrackIterator(this, 0);
  }

  Iterator path_begin(std::size_t path_ord) const {
    return Iterator(this, path_ord);
  }

  Iterator path_end() const {
    return Iterator(this);
  }

private:
  inline const TClosure& closure() const {
    return sp_range_query_->closure();
  }

  template<typename T2>
  struct Node {
    Node(const Edge<T2>& last_sidetrack, std::size_t prefpath_ord)
    : last_sidetrack(last_sidetrack),
      prefpath_ord(prefpath_ord) { }

    Edge<T2> last_sidetrack;
    std::size_t prefpath_ord;
  };

  VertexId start_vertex_;
  const ShortestPathTree<T>* sp_tree_;
  const ShortestPathRangeQuery<T, TClosure>* sp_range_query_;

  std::vector<Node<T> > paths_;
  std::vector<T> d_;

  friend class EppsteinPathTreeIterator<T, TClosure>;
  friend class PathIterator<T, TClosure>;
};


template<typename T = unsigned, class TClosure = std::plus<T> >
class EppsteinPathTreeIterator : public std::iterator<std::forward_iterator_tag,
ptrdiff_t> {
public:
  EppsteinPathTreeIterator() : path_tree_(NULL), path_ord_(0) { }

  inline const Edge<T>& operator*() const {
    return path_tree_->paths_[path_ord_].last_sidetrack;
  }

  inline const Edge<T>& operator->() const {
    return &path_tree_->paths_[path_ord_].last_sidetrack;
  }

  inline EppsteinPathTreeIterator& operator++() {
    path_ord_ = path_tree_->paths_[path_ord_].prefpath_ord;
    return *this;
  }

  inline EppsteinPathTreeIterator operator++(int) {
    EppsteinPathTreeIterator old(*this);
    path_ord_ = path_tree_->paths_[path_ord_].prefpath_ord;
    return old;
  }

  inline friend bool operator==(const EppsteinPathTreeIterator& lhs,
      const EppsteinPathTreeIterator& rhs) {
    return lhs.path_ord_ == rhs.path_ord_ && lhs.path_tree_ == rhs.path_tree_;
  }

  inline friend bool operator!=(const EppsteinPathTreeIterator& lhs,
      const EppsteinPathTreeIterator& rhs) {
    return !(lhs == rhs);
  }

private:
  typedef EppsteinPathTree<T, TClosure> PathTreeType;

  EppsteinPathTreeIterator(const PathTreeType* path_tree, std::size_t path_ord)
  : path_tree_(path_tree),
    path_ord_(path_ord) { }

  const PathTreeType* path_tree_;
  std::size_t path_ord_;

  friend class EppsteinPathTree<T, TClosure>;
};


template<typename T = unsigned, class TClosure = std::plus<T> >
class PathIterator {
public:
  PathIterator() { }

  inline Edge<T> operator*() const {
    if (sp_tree_it_ != sp_tree_it_end_) {
      Edge<T> e(*sp_tree_it_); e.Reverse();
      return e;
    }
    return path_tree_->paths_[path_ords_.top()].last_sidetrack;
  }

  inline PathIterator& operator++() {
    if (sp_tree_it_ != sp_tree_it_end_)
      ++sp_tree_it_;
    else {
      sp_tree_it_ = path_tree_->sp_tree_->path_rbegin(
          path_tree_->paths_[path_ords_.top()].last_sidetrack.head);
      path_ords_.pop();
      UpdateShortestPathEnd();
    }
    return *this;
  }

  inline PathIterator operator++(int) {
    PathIterator old(*this);
    ++(*this);
    return old;
  }

  inline friend bool operator==(const PathIterator& lhs,
      const PathIterator& rhs) {
    return lhs.path_ords_.top() == rhs.path_ords_.top() &&
        lhs.sp_tree_it_ == rhs.sp_tree_it_ && lhs.path_tree_ == rhs.path_tree_;
  }

  inline friend bool operator!=(const PathIterator& lhs,
        const PathIterator& rhs) {
    return !(lhs == rhs);
  }

private:
  typedef EppsteinPathTree<T, TClosure> PathTreeType;

  PathIterator(const PathTreeType* path_tree, std::size_t path_ord)
  : path_tree_(path_tree),
    sp_tree_it_(path_tree->sp_tree_->path_rbegin(path_tree->start_vertex())) {
    InitPathOrdStack();
    while (path_ord) {
      path_ords_.push(path_ord);
      path_ord = path_tree->paths_[path_ord].prefpath_ord;
    }
    UpdateShortestPathEnd();
  }

  PathIterator(const PathTreeType* path_tree)
  : path_tree_(path_tree),
    sp_tree_it_(path_tree->sp_tree_->path_rend(path_tree_->end_vertex())),
    sp_tree_it_end_(sp_tree_it_) {
    InitPathOrdStack();
  }

  void UpdateShortestPathEnd() {
    sp_tree_it_end_ = (path_ords_.top() == -1U)
      ? path_tree_->sp_tree_->path_rend(path_tree_->end_vertex())
      : path_tree_->sp_tree_->path_rbegin(
          path_tree_->paths_[path_ords_.top()].last_sidetrack.tail);
  }

  void InitPathOrdStack() {
    path_ords_.push(-1U);  // sentinel path ordinal
  }

  const PathTreeType* path_tree_;
  std::stack<std::size_t> path_ords_;
  typename ShortestPathTree<T>::Iter sp_tree_it_;
  typename ShortestPathTree<T>::Iter sp_tree_it_end_;

  friend class EppsteinPathTree<T, TClosure>;
};

}  // namespace ksp

#endif /* EPPSTEIN_PATH_TREE_HPP_ */
