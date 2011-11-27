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

#ifndef EPPSTEIN_PATH_FINDER_HPP_
#define EPPSTEIN_PATH_FINDER_HPP_

#include <functional>
#include <queue>

#include "path_graph.hpp"
#include "path_tree.hpp"

namespace ksp {

template<typename T = unsigned, class TCompare = std::less<T>,
    class TClosure = std::plus<T> >
class EppsteinKShortestPathFinder {
  template<typename T2, class TCompare2, class TClosure2>
  struct PriorityQueueNode;
  typedef PriorityQueueNode<T, TCompare, TClosure> Node;
  template<typename T2, class TCompare2, class TClosure2>
  class PriorityQueueNodeComp;
  typedef PriorityQueueNodeComp<T, TCompare, TClosure> NodeComp;
public:
  typedef EppsteinPathGraph<T, TCompare, TClosure> PathGraphType;
  typedef typename EppsteinPathGraph<T, TCompare, TClosure>::Locator Locator;
  typedef EppsteinPathTree<T, TClosure> PathTreeType;

  EppsteinKShortestPathFinder(const PathGraphType& p, PathTreeType* path_tree)
  : p_(p),
    path_tree_(path_tree),
    q_(NodeComp(p.compare())) {
    const typename PathGraphType::Locator& start_loc = p_.locator(
        path_tree_->start_vertex());
    if (!p_.is_end(start_loc))
      q_.push(Node(p, start_loc));
  }

  bool FindNext() {
    if (q_.empty())
      return false;

    Node u = q_.top(); q_.pop();
    std::size_t prefpath_ord = path_tree_->AddPath(p_.data(u.loc).edge,
                                                   u.prefpath_ord);
    if (!p_.is_end(u.loc)) {
      q_.push(Node(p_, prefpath_ord, u));  // take cross edge
      for (unsigned i = p_.heap_edge_count(u.loc); i--; ) {  // take heap edges
        q_.push(Node(p_, u, i));
      }
    }
    return true;
  }

  bool Find(std::size_t k) {
    while (this->k() < k)
      if (!FindNext())
        return false;
    return true;
  }

  const PathTreeType& path_tree() const {
    return *path_tree_;
  }

  std::size_t k() const {
    return path_tree_->k();
  }

private:
  template<typename T2, class TCompare2, class TClosure2>
  struct PriorityQueueNode {
    typedef EppsteinPathGraph<T2, TCompare2, TClosure2> PathGraphType;

    PriorityQueueNode(const PathGraphType& p,
                      const typename PathGraphType::Locator& start_vertex_loc)
    : loc(start_vertex_loc),
      prefpath_ord(0),
      val(p.data(loc).val) { }

    PriorityQueueNode(const PathGraphType& p, const PriorityQueueNode& parent,
                      unsigned heap_edge_index)
    : loc(parent.loc),
      prefpath_ord(parent.prefpath_ord),
      val(p.closure()(parent.val, p.Descend(heap_edge_index, &loc))) { }

    PriorityQueueNode(const PathGraphType& p, std::size_t prefpath_ord,
                      const PriorityQueueNode& tail)
    : loc(tail.loc),
      prefpath_ord(prefpath_ord),
      val(p.closure()(tail.val, p.Cross(&loc))) { }

    Locator loc;
    std::size_t prefpath_ord;
    T2 val;
  };

  template<typename T2, class TCompare2, class TClosure2>
  class PriorityQueueNodeComp {
  public:
    PriorityQueueNodeComp(const TCompare2& comp) : comp_(comp) { }

    bool operator()(
        const PriorityQueueNode<T2, TCompare2, TClosure2>& lhs,
        const PriorityQueueNode<T2, TCompare2, TClosure2>& rhs) const {
      return comp_(rhs.val, lhs.val);
    }
  private:
    TCompare2 comp_;
  };

  const PathGraphType& p_;
  PathTreeType* path_tree_;
  std::priority_queue<Node, std::vector<Node>, NodeComp> q_;
};

}  // namespace ksp

#endif /* EPPSTEIN_PATH_FINDER_HPP_ */
