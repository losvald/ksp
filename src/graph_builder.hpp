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

#ifndef GRAPH_BUILDER_HPP_
#define GRAPH_BUILDER_HPP_

#include <functional>
#include <iostream>
#include <map>
#include <vector>

#include "edge.hpp"
#include "edge_map.hpp"
#include "vertex.hpp"
#include "vertex_map.hpp"

namespace ksp {

template<typename V, typename T = unsigned, class VCompare = std::less<V> >
class GraphBuilder {
public:
  typedef V VertexType;
  typedef std::map<V, VertexId, VCompare> VertexIdMap;
  typedef Edge<T> EdgeType;
  typedef EdgeMap<EdgeType> EdgeMapType;

  GraphBuilder() { }

  GraphBuilder(unsigned vertex_capacity, unsigned edge_capacity = -1U) {
    vertices_.reserve(vertex_capacity);
    edges_.reserve(edge_capacity != -1U ? edge_capacity : vertex_capacity);
  }

  friend std::ostream& operator<<(std::ostream& os, const GraphBuilder& g) {
    using std::endl;
    os << "|V| = " << g.vertex_count() << endl;
    os << "|E| = " << g.edge_count() << endl;
    for (typename EdgeMapType::const_iterator e = g.edges_.begin();
        e != g.edges_.end(); ++e) {
      os << g.vertex(e->tail) << " -> " << g.vertex(e->head) << ": " <<
          e->data << endl;
    }
    os << endl;
    return os;
  }

  VertexId AddVertex(const V& v) {
    if (!vertex_ids_.count(v)) {
      VertexId v_id = vertex_ids_.size();
      vertex_ids_.insert(std::make_pair(v, v_id));
      vertices_.push_back(v);
      return v_id;
    }
    return kNullVertexId;
  }

  EdgeId AddEdge(const V& tail, const V& head, const T& edge_data) {
    if (!vertex_ids_.count(tail) || !vertex_ids_.count(head))
      throw NoSuchVertexException();
    EdgeId e_id = edges_.size();
    edges_.push_back(Edge<T>(vertex_ids_.at(tail), vertex_ids_.at(head),
                             edge_data));
    return e_id;
  }

  void Clear() {
    vertex_ids_.clear();
    vertices_.clear();
    edges_.clear();
  }

  void Transpose() {
    for (typename EdgeMapType::iterator e = edges_.begin(), last = edges_.end();
        e != last; ++e)
      e->Reverse();
  }

  std::size_t vertex_count() const {
    return vertex_ids_.size();
  }

  std::size_t edge_count() const {
    return edges_.size();
  }

  const V& vertex(VertexId vertex_id) const {
    return vertices_.at(vertex_id);
  }

  VertexId vertex_id(const V& vertex) const {
    if (!vertex_ids_.count(vertex))
      throw NoSuchVertexException();
    return vertex_ids_.at(vertex);
  }

  const EdgeType& edge(EdgeId edge_id) const {
    return edges_.at(edge_id);
  }

  typename EdgeMapType::const_iterator edges_begin() const {
    return edges_.begin();
  }

  typename EdgeMapType::const_iterator edges_end() const {
    return edges_.end();
  }

  VertexIdIterator vertices_begin() const {
    return VertexIdIterator(0U);
  }

  VertexIdIterator vertices_end() const {
    return VertexIdIterator(vertices_.size());
  }

private:
  VertexIdMap vertex_ids_;
  VertexMap<V> vertices_;
  EdgeMapType edges_;
};

}  // namespace ksp

#endif /* GRAPH_BUILDER_HPP_ */
