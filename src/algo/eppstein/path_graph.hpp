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

#ifndef EPPSTEIN_PATH_GRAPH_HPP_
#define EPPSTEIN_PATH_GRAPH_HPP_

#include <algorithm>
#include <functional>
#include <limits>
#include <sstream>  // TODO
#include <vector>
#include <iostream>  // TODO

#include "../../edge.hpp"
#include "../../shortest_path_tree.hpp"
#include "../../vertex.hpp"
#include "../../vertex_map.hpp"

#include "../dijkstra.hpp"
#include "../sp_range_query.hpp"
#include "../topological_sort.hpp"

#include "path_tree.hpp"

namespace ksp {

namespace {

template<class Predicate>
class ReverseBinaryFunction : std::binary_function<
  typename Predicate::first_argument_type,
  typename Predicate::second_argument_type,
  typename Predicate::result_type> {
public:
  ReverseBinaryFunction(const Predicate& pred) : pred_(pred) { }

  inline typename Predicate::result_type operator()(
      const typename Predicate::second_argument_type& lhs,
      const typename Predicate::first_argument_type& rhs) const {
    return pred_(rhs, lhs);
  }

private:
  Predicate pred_;
};

template<typename T>
class HeapDataComparator;

template<typename T>
struct HeapData {
  HeapData(const T& val, const Edge<T>& edge) : val(val), edge(edge) { }

  T val;
  Edge<T> edge;

  friend class HeapDataComparator<T>;
};

template<class Predicate>
class HeapDataComparator : std::binary_function<
    typename Predicate::first_argument_type,
    typename Predicate::second_argument_type,
    typename Predicate::result_type> {
public:
  HeapDataComparator(const Predicate& pred) : pred_(pred) { }

  inline typename Predicate::result_type operator()(
      const HeapData<typename Predicate::first_argument_type>& lhs,
      const HeapData<typename Predicate::second_argument_type>& rhs) const {
    return pred_(lhs.val, rhs.val);
  }

private:
  ReverseBinaryFunction<Predicate> pred_;
};


template<typename T>
class MergerHeap {
  template<typename T2>
  struct MergerHeapNode;
public:
  typedef MergerHeapNode<T> Node;

  MergerHeap() : root_(NULL), size_(0), child_size_(0) { }

  MergerHeap(const MergerHeap<T>& clone) {
    Clone(clone);
  }

  ~MergerHeap() {
    if (size_ == child_size_)
      return ;
    DeleteAlongPath(path_dirs_);
  }

  MergerHeap<T>& operator=(const MergerHeap<T>& clone) {
    if (this != &clone) {
      Clone(clone);
    }
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os, const MergerHeap& h) {
    std::stringstream oss;
    h.ToStringRec(h.root_, oss);
    os << oss.str();
    return os;
  }

  void Merge(const HeapData<T>& data) {
    root_ = new MergerHeapNode<T>(data);
    size_ = 1;
    child_size_ = 0;
    path_dirs_.clear();
  }

  void Merge(const MergerHeap& child) {
    root_ = child.root_;
    size_ = child.size_;
    child_size_ = child.size_;
    path_dirs_.clear();
  }

  template<class TCompare, class TWeightDiff>
  void Merge(const MergerHeap& child, const HeapData<T>& data,
             const TCompare& compare,
             const TWeightDiff& weight_diff) {
    size_ = child.size_ + 1;
    child_size_ = child.size_;
    if (child.size_ == 0) {
      root_ = new MergerHeapNode<T>(data);
      path_dirs_.clear();
      return ;
    }

    // compute path for insertion of the new node
    ComputePathDirs(size_, &path_dirs_);

    // duplicate nodes along that path
    MergerHeapNode<T>* p = DuplicateAlongPath(child, path_dirs_);
    MergerHeapNode<T>* n = p->c[path_dirs_.back()] =
        new MergerHeapNode<T>(data, p);

    // correct heap by swapping data along the update path
    if (compare(data.val, p->data.val)) {
      do {
        n->data = p->data;
        n = p;
      } while ((p = p->parent) != NULL && compare(data.val, p->data.val));
      n->data = data;
    }
//    while (p != NULL && compare(data.val, p->data.val)) {
//      n->data = p->data;
//      n = p;
//      p = p->parent;
//    }
//    n->data = data;

    UpdateEdgeWeights(path_dirs_, weight_diff);
  }

  inline const MergerHeapNode<T>* root() const {
    return root_;
  }

private:
//  MergerHeap<T>& operator=(const MergerHeap<T>&);

  MergerHeapNode<T>* DuplicateAlongPath(const MergerHeap<T>& child,
                                        const std::vector<bool>& path_dirs) {
    MergerHeapNode<T>* p = root_ = new MergerHeapNode<T>(*child.root_);
    const MergerHeapNode<T>* child_n = child.root_;
    for (std::vector<bool>::const_iterator dir = path_dirs.begin(),
        dir_end = --path_dirs.end(); dir != dir_end; ++dir) {
      child_n = child_n->c[*dir];
      p = p->c[*dir] = new MergerHeapNode<T>(*child_n, p);
    }
    return p;
  }

  void DeleteAlongPath(const std::vector<bool>& path_dirs) {
    MergerHeapNode<T>* p = root_;
    for (std::vector<bool>::const_iterator dir = path_dirs.begin(),
        dir_end = path_dirs.end(); dir != dir_end; ++dir) {
      MergerHeapNode<T>* n = p->c[*dir];
      delete p;
      p = n;
    }
    delete p;
    root_ = NULL;
  }

  template<class TWeightDiff>
  void UpdateEdgeWeights(const std::vector<bool>& path_dirs,
                         const TWeightDiff& weight_diff) {
    MergerHeapNode<T>* p = root_;
    for (std::vector<bool>::const_iterator dir = path_dirs.begin(),
        dir_end = path_dirs.end(); dir != dir_end; ++dir) {
      MergerHeapNode<T>* p_next_sib = p->c[!*dir];
      if (p_next_sib != NULL)
        p->diff[!*dir] = weight_diff(p_next_sib->data.val, p->data.val);
      MergerHeapNode<T>* p_next = p->c[*dir];
      p->diff[*dir] = weight_diff(p_next->data.val, p->data.val);
      p = p_next;
    }
  }

  void Clone(const MergerHeap<T>& clone) {
    root_ = clone.root_;
    size_ = clone.size_;
    child_size_ = clone.child_size_;
    path_dirs_ = clone.path_dirs_;
    if (clone.child_size_)
      DuplicateAlongPath(clone, path_dirs_);
  }

  void ToStringRec(const MergerHeapNode<T>* node,
                   std::stringstream& oss,
                   std::size_t level = 0,
                   bool dup = true) const {
    oss << "(";
    if (node != NULL) {
      oss << node->data.val << (dup ? "*" : "") << " ";
//      EdgeWeightToStr(node->diff[0], oss);
      ToStringRec(node->c[0], oss, level + 1,
                  dup && level < path_dirs_.size() && !path_dirs_[level]);
      oss << " ";
//      EdgeWeightToStr(node->diff[1], oss);
      ToStringRec(node->c[1], oss, level + 1,
                  dup && level < path_dirs_.size() && path_dirs_[level]);
    }
    oss << ")";
  }

  static void EdgeWeightToStr(const T& edge_weight, std::stringstream& oss) {
    oss << "[" << edge_weight << "]";
  }

  static void ComputePathDirs(std::size_t pos, std::vector<bool>* path_dirs) {
    std::size_t h = 0;
    for (std::size_t tmp = pos; tmp > 1; tmp >>= 1)
      ++h;
    path_dirs->resize(h);
    for (std::size_t tmp = pos; tmp > 1; tmp >>= 1)
      (*path_dirs)[--h] = tmp & 1;
  }

  template<typename T2>
  struct MergerHeapNode {
    MergerHeapNode(const HeapData<T2>& data, MergerHeapNode* parent = NULL)
    : data(data),
      parent(parent) {
      c[0] = c[1] = NULL;
      diff[0] = diff[1] = T2();
    }

    MergerHeapNode(const MergerHeapNode& clone, MergerHeapNode* parent)
    : data(clone.data),
      parent(parent) {
      c[0] = clone.c[0];
      c[1] = clone.c[1];
      diff[0] = diff[1] = T2();
    }

    inline unsigned children_count() const {
      return (c[0] != NULL) + (c[1] != NULL);
    }

    HeapData<T2> data;
    MergerHeapNode* parent;
    MergerHeapNode* c[2];
    T2 diff[2];
  };

  Node* root_;
  std::size_t size_;
  std::size_t child_size_;
  std::vector<bool> path_dirs_;
};

template<typename T, class TCompare>
static inline bool EqualEdgeData(const T& lhs, const T& rhs,
                                 const TCompare& compare) {
  return !compare(lhs, rhs) && !compare(rhs, lhs);
}


template<typename T>
class OutHeap {
public:
  const HeapData<T>& operator[](std::size_t n) const {
    return a_[n];
  }

  template<class InputIterator, class HeapCompare, class TEquals,
  class TClosure, class TWeightDiff>
  void Init(InputIterator adj_edges_begin, InputIterator adj_edges_end,
            VertexId v, const ShortestPathTree<T>& sp_tree,
            const HeapCompare& heap_comp,
            const TEquals& equals = std::equal_to<T>(),
            const TClosure& closure = std::plus<T>(),
            const TWeightDiff& weight_diff = std::minus<T>()) {
    const VertexMap<T>& d = sp_tree.distance_map();
    const Edge<T>& t_edge = *sp_tree.path_rbegin(v);
    a_.reserve(std::distance(adj_edges_begin, adj_edges_end));
    for (; adj_edges_begin != adj_edges_end; ++adj_edges_begin) {
      if (sp_tree.is_reachable(adj_edges_begin->head) &&
          (adj_edges_begin->head != t_edge.tail ||
              !equals(adj_edges_begin->data, t_edge.data))) {
        a_.push_back(HeapData<T>(weight_diff(
            closure(adj_edges_begin->data, d[adj_edges_begin->tail]),
            d[adj_edges_begin->head]),
            *adj_edges_begin));
      }
    }

    if (!a_.empty()) {
      std::swap(*std::max_element(a_.begin(), a_.end(), heap_comp), a_.front());
      std::make_heap(++a_.begin(), a_.end(), heap_comp);
      for (std::size_t p = a_.size() - 1; p; --p)
        a_[p].val = weight_diff(a_[p].val, a_[p >> 1].val);
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const OutHeap& h) {
    std::stringstream oss;
    h.ToStringRec(0, oss);
    os << oss.str();
    return os;
  }

  bool empty() const {
    return a_.empty();
  }

  std::size_t size() const {
    return a_.size();
  }

  const HeapData<T>& root() const {
    return a_[0];
  }

  const HeapData<T>& root_child() const {
    return a_[1];
  }

private:
  void ToStringRec(std::size_t pos,
                   std::stringstream& oss) const {
    oss << "(";
    if (pos < a_.size()) {
      oss << a_[pos].val << " ";
      if (pos == 0) {
        oss << "() ";
        ToStringRec(1, oss);
      } else {
        ToStringRec(2 * pos + !pos, oss);
        oss << " ";
        ToStringRec(2 * pos + 1, oss);
      }
    }
    oss << ")";
  }

  std::vector<HeapData<T> > a_;
};

}  // namespace

template<typename T, class TCompare, class TClosure>
class EppsteinPathGraph;

template<typename T, class TCompare, class TClosure>
class EppsteinPathGraphLocator {
public:
  friend bool operator==(const EppsteinPathGraphLocator& lhs,
      const EppsteinPathGraphLocator& rhs) {
    return lhs.h_out_pos_ == rhs.h_out_pos_ &&
        ((lhs.h_out_pos_ && lhs.v() == rhs.v()) ||
            lhs.h_t_node_ == rhs.h_t_node_);
  }

  friend bool operator!=(const EppsteinPathGraphLocator& lhs,
        const EppsteinPathGraphLocator& rhs) {
    return !(lhs == rhs);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const EppsteinPathGraphLocator& l) {
    os << "(" << (int)l.h_t_node_ << ": " << l.h_out_pos_ << ")";
    return os;
  }

private:
  EppsteinPathGraphLocator(const typename MergerHeap<T>::Node* h_t_node)
  : h_out_pos_(0),
    h_t_node_(h_t_node) { }

  inline const VertexId v() const {
    return h_t_node_->data.edge.tail;
  }

  std::size_t h_out_pos_;
  const typename MergerHeap<T>::Node* h_t_node_;

  friend class EppsteinPathGraph<T, TCompare, TClosure>;
};


template<typename T = unsigned, class TCompare = std::less<T>,
    class TClosure = std::plus<T> >
class EppsteinPathGraph {
public:
  typedef EppsteinPathGraphLocator<T, TCompare, TClosure> Locator;

  EppsteinPathGraph(const TCompare& comp = TCompare(),
            const TClosure& closure = TClosure())
  : comp_(comp),
    sp_range_query_(closure) { }

  template<class TEquals, class TWeightDiff>
  EppsteinPathGraph(const AdjacencyList<T>& adj, VertexId end_vertex,
                    const TEquals& equals,
                    const TWeightDiff& weight_diff,
                    const T& distance_max = std::numeric_limits<T>::max(),
                    const TCompare& comp = TCompare(),
                    const TClosure& closure = TClosure())
  : comp_(comp),
    sp_range_query_(closure) {
    std::vector<Edge<T> > g_r(adj.edges_begin(), adj.edges_end());
    for (typename std::vector<Edge<T> >::iterator e = g_r.begin(),
        e_end = g_r.end(); e != e_end; ++e) {
      e->Reverse();
    }
    DijkstraShortestPaths(g_r.begin(), g_r.end(), end_vertex, &sp_tree_,
                          distance_max, comp_, this->closure());
    g_r.clear();
    InitFromSpTree(adj, equals, weight_diff);
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const EppsteinPathGraph& p) {
    for (VertexId v = 0; v < p.h_t_.size(); ++v) {
      os << "H_T(" << v << ") = " << p.h_t_[v] << std::endl <<
          "H_out(" << v << ") = " << p.h_out_[v] << std::endl;
    }
    return os;
  }

  template<class TEquals, class TWeightDiff>
  void Init(const AdjacencyList<T>& adj, const ShortestPathTree<T>& sp_tree,
            const TEquals& equals = std::equal_to<T>(),
            const TWeightDiff& weight_diff = std::minus<T>()) {
    sp_tree_ = sp_tree;
    InitFromSpTree(adj, equals, weight_diff);
  }

  T Descend(std::size_t n_child, Locator* const l) const {
    if (!l->h_out_pos_) {
      std::size_t h_t_children_cnt = l->h_t_node_->children_count();
      if (n_child < h_t_children_cnt) {  // use h_t
        const T& heap_edge_weight = l->h_t_node_->diff[n_child];
        l->h_t_node_ = l->h_t_node_->c[n_child];
        l->h_out_pos_ = 0;
        return heap_edge_weight;
      }
      n_child -= h_t_children_cnt;
    }  // use h_out
    l->h_out_pos_ = (l->h_out_pos_ << 1) + !l->h_out_pos_ + n_child;
    return h_out_node(*l).val;
  }

  T Cross(Locator* const l) const {
    l->h_t_node_ = h_t_[data(*l).edge.head].root();
    l->h_out_pos_ = 0;
    return l->h_t_node_ != NULL ? l->h_t_node_->data.val : T();
  }

  inline Locator locator(VertexId v) const {
    return Locator(h_t_[v].root());
  }

  inline const HeapData<T>& data(const Locator& l) const {
    return (l.h_out_pos_ ? h_out_node(l) : l.h_t_node_->data);
  }

  std::size_t heap_edge_count(const Locator& l) const {
    std::size_t h_out_size_l = h_out_size(l);
    return l.h_out_pos_
        ? (l.h_out_pos_ << 1 < h_out_size_l) +
        ((l.h_out_pos_ << 1) + 1 < h_out_size_l)
        : l.h_t_node_->children_count() + (h_out_size_l > 1);
  }

  inline const TCompare& compare() const {
    return comp_;
  }

  inline const TClosure& closure() const {
    return sp_range_query_.closure();
  }

  inline const ShortestPathTree<T>& shortest_path_tree() const {
    return sp_tree_;
  }

  EppsteinPathTree<T, TClosure> path_tree(VertexId start_vertex) const {
    return EppsteinPathTree<T, TClosure>(&sp_range_query_, &sp_tree_,
                                         start_vertex);
  }

  inline bool is_end(const Locator& l) const {
    return l.h_t_node_ == NULL;
  }

private:
  template<class TEquals, class TWeightDiff>
  void InitFromSpTree(const AdjacencyList<T>& adj, const TEquals& equals,
                      const TWeightDiff& weight_diff) {
    HeapDataComparator<TCompare> h_out_comp(comp_);
    h_t_.clear();
    h_t_.resize(adj.max_vertex_id() + 1);
    h_out_.clear();
    h_out_.resize(adj.max_vertex_id() + 1);

    std::vector<VertexId> v_toposorted;
    TopologicalSort<T>(sp_tree_.edges_begin(), sp_tree_.edges_end(),
                       &v_toposorted);
    for (typename std::vector<VertexId>::const_iterator v_it =
        v_toposorted.begin(), v_end = v_toposorted.end();
        v_it != v_end; ++v_it) {
      VertexId v = *v_it;
      if (!sp_tree_.is_reachable(v))
        continue;
      h_out_[v].Init(adj.first(v), adj.last(v), v, sp_tree_, h_out_comp,
                     equals, sp_range_query_.closure(), weight_diff);
      VertexId v_next = sp_tree_.path_rbegin(v)->tail;
      if (v_next != kNullVertexId) {
        if (!h_out_[v].empty())
          h_t_[v].Merge(h_t_[v_next], h_out_[v].root(), comp_, weight_diff);
        else
          h_t_[v].Merge(h_t_[v_next]);
      } else if (!h_out_[v].empty())
        h_t_[v].Merge(h_out_[v].root());
    }
    sp_range_query_.Init(sp_tree_, v_toposorted.begin(), v_toposorted.end());
  }

  inline const HeapData<T>& h_out_node(const Locator& l) const {
    return h_out_[l.h_t_node_->data.edge.tail][l.h_out_pos_];
  }

  inline std::size_t h_out_size(const Locator& l) const {
    return h_out_[l.h_t_node_->data.edge.tail].size();
  }

  TCompare comp_;
  ShortestPathRangeQuery<T, TClosure> sp_range_query_;

  VertexMap<MergerHeap<T> > h_t_;
  VertexMap<OutHeap<T> > h_out_;
  ShortestPathTree<T> sp_tree_;
};

}  // namespace ksp

#endif /* EPPSTEIN_PATH_GRAPH_HPP_ */
