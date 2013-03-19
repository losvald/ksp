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

#ifndef GRAPH_IO_HPP_
#define GRAPH_IO_HPP_

#include <algorithm>
#include <iostream>
#include <iterator>

#include "../edge_map.hpp"
#include "../graph_builder.hpp"

namespace ksp {

namespace {

template<typename T>
inline void ReadEdgeData(std::istream& is, T& data) {
  is >> data;
}

template<>
inline void ReadEdgeData(std::istream& is, Unweighted& data) {
}

} // namespace


template<typename V, typename T, class VCompare>
void ReadGraph(std::istream& is,
               GraphBuilder<V, T, VCompare>* g) {
  g->Clear();
  std::size_t n, m;
  is >> n >> m;
  for (std::size_t i = 0; i < n; ++i) {
    V v;
    is >> v;
    g->AddVertex(v);
  }
  for (std::size_t i = m; i != 0; --i) {
    V tail, head;
    T edge_data;
    is >> tail >> head;
    ReadEdgeData<T>(is, edge_data);
    g->AddEdge(tail, head, edge_data);
  }
}

template<typename T, typename V = VertexId, class VCompare = std::less<V> >
class GraphReader {
 public:
  typedef EdgeMap<Edge<T> > EdgeMapType;
  typedef typename EdgeMapType::const_iterator EdgeIter;

  GraphReader(std::istream& is) : is_(is) {
  }

  virtual bool ReadNext() = 0;

  virtual std::size_t vertex_count() const = 0;

  std::size_t edge_count() const {
    return std::distance(edges_begin(), edges_end());
  }

  virtual EdgeIter edges_begin() const = 0;

  virtual EdgeIter edges_end() const = 0;

  VertexIdIterator vertices_begin() const {
    return VertexIdIterator(0U);
  }

  VertexIdIterator vertices_end() const {
    return VertexIdIterator(vertex_count());
  }

 protected:
  std::istream& is_;
};

template<typename T = Unweighted>
class BasicGraphReader : public GraphReader<T> {
 public:
  BasicGraphReader(std::istream& is) : GraphReader<T>(is) {
  }

  bool ReadNext() {
    std::size_t n, m;
    this->is_ >> n >> m;
    if (this->is_.eof())
      return false;

    max_vertex_id_ = (m ? 0 : kNullVertexId);
    edges_.clear();
    edges_.reserve(m);
    while (m--) {
      VertexId tail, head;
      T edge_data;
      this->is_ >> tail >> head;
      ReadEdgeData<T>(this->is_, edge_data);
      max_vertex_id_ = std::max(max_vertex_id_, std::max(tail, head));
      edges_.push_back(Edge<T>(tail, head));
    }
    return true;
  }

  std::size_t vertex_count() const {
    return max_vertex_id_ + 1;
  }

  typename BasicGraphReader::EdgeIter edges_begin() const {
    return edges_.begin();
  }

  typename BasicGraphReader::EdgeIter edges_end() const {
    return edges_.end();
  }

 private:
  typename BasicGraphReader::EdgeMapType edges_;
  VertexId max_vertex_id_;
};

template<typename V, typename T = Unweighted, class VCompare = std::less<V> >
class DefaultGraphReader : public GraphReader<T, V, VCompare> {
 public:
  DefaultGraphReader(std::istream& is, std::size_t vertex_capacity = 8U)
      : GraphReader<T, V, VCompare>(is),
        builder_(vertex_capacity),
        vertex_capacity_(vertex_capacity) { }

  bool ReadNext() {
    ReadGraph(this->is_, &builder_);
    return this->is_.eof();
  }

  std::size_t vertex_count() const {
    return builder_.vertex_count();
  }

  typename DefaultGraphReader::EdgeIter edges_begin() const {
    return builder_.edges_begin();
  }

  typename DefaultGraphReader::EdgeIter edges_end() const {
    return builder_.edges_end();
  }

 private:
  GraphBuilder<V, T, VCompare> builder_;
  std::size_t vertex_capacity_;
};


}  // namespace ksp

#endif /* GRAPH_IO_HPP_ */
