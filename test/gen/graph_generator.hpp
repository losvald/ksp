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

#ifndef GRAPH_GENERATOR_HPP_
#define GRAPH_GENERATOR_HPP_

#include <exception>
#include <map>

#include "../../src/edge.hpp"
#include "../../src/vertex.hpp"

#include "discrete_random_variable.hpp"
#include "edge_generator.hpp"
#include "generator.hpp"
#include "random_sampler.hpp"
#include "random_utils.hpp"

namespace ksp {

namespace gen {

namespace test {

namespace {

inline std::size_t sum_first(std::size_t n) {
  return n * (n - 1) / 2;
}

} // namespace

//class GraphGeneratorException : std::exception {
//};
//
//class NoMoreElementsException : GraphGeneratorException {
//};

template<class TGenerator, typename T = unsigned>
class SimpleWeightedGraphGenerator :
  public ::gen::test::Generator<
  SimpleWeightedGraphGenerator<TGenerator, T>, Edge<T> > {
  typedef ::gen::test::Generator<
    SimpleWeightedGraphGenerator<TGenerator, T>, Edge<T> > Base;

public:
  SimpleWeightedGraphGenerator(unsigned vertex_count,
                               const TGenerator& edge_data_gen = TGenerator(),
                               unsigned seed = ::gen::test::NewSeedVal())
      : Base(seed, vertex_count * (vertex_count - 1U)),
        adj_vtx_cnt_(vertex_count - 1U),
        sampler_(vertex_count * adj_vtx_cnt_, seed),
        edge_data_gen_(edge_data_gen) {
  }

  Edge<T> operator()() {
    std::size_t ind = sampler_();
    if (ind == -1U)
      return NullEdge<T>();

    VertexId tail = ind / adj_vtx_cnt_, head = ind - tail * adj_vtx_cnt_;
    head += (tail <= head);
    return Edge<T>(tail, head, edge_data_gen_());
  }

private:
  std::size_t adj_vtx_cnt_;
  ::gen::test::RandomSampler sampler_;
  TGenerator edge_data_gen_;
};

template<class TGenerator, typename T = unsigned>
class SimpleWeightedUndirectedGraphGenerator :
  public ::gen::test::Generator<
  SimpleWeightedUndirectedGraphGenerator<TGenerator, T>, Edge<T> > {
  typedef ::gen::test::Generator<
    SimpleWeightedUndirectedGraphGenerator<TGenerator, T>, Edge<T> > Base;

public:
  SimpleWeightedUndirectedGraphGenerator(
      unsigned vertex_count,
      const TGenerator& edge_data_gen = TGenerator(),
      unsigned seed = ::gen::test::NewSeedVal())
      : Base(seed, sum_first(vertex_count)),
        sampler_(sum_first(vertex_count), seed),
        vtx_cnt_(vertex_count),
        edge_data_gen_(edge_data_gen) {
  }

  Edge<T> operator()() {
    std::size_t ind = sampler_();
    if (ind == -1U)
      return NullEdge<T>();

    VertexId head = GetAdjMatRow(ind), tail = ind - sum_first(head);
    return Edge<T>(tail, head, edge_data_gen_());
  }

private:
  VertexId GetAdjMatRow(std::size_t ind) {
    // 0: x
    // 1: 0 x
    // 2: 1 2 x
    // 3: 3 4 5 x
    // 4: 6 7 8 9 x
    VertexId lo = 1, hi = vtx_cnt_;
    while (lo < hi) {
      VertexId mid = (lo + hi + 1) / 2;
      if (sum_first(mid) <= ind)
        lo = mid;
      else
        hi = mid - 1;
    }
    return lo;
  }

  ::gen::test::RandomSampler sampler_;
  std::size_t vtx_cnt_;
  TGenerator edge_data_gen_;
};

class UndirectedEdgeGenerator {
 public:
  inline void* operator()() const {
    return NULL;
  }
};


class SimpleGraphGenerator
    : public SimpleWeightedGraphGenerator<UndirectedEdgeGenerator, void*> {
  typedef SimpleWeightedGraphGenerator<UndirectedEdgeGenerator, void*> super;

 public:
  SimpleGraphGenerator(unsigned vertex_count,
                       unsigned seed = ::gen::test::NewSeedVal())
      : super(vertex_count, UndirectedEdgeGenerator(), seed) { }
};

class SimpleUndirectedGraphGenerator
    : public SimpleWeightedUndirectedGraphGenerator<UndirectedEdgeGenerator,
                                                    void*> {
  typedef SimpleWeightedUndirectedGraphGenerator<UndirectedEdgeGenerator,
                                                 void*> super;

 public:
  SimpleUndirectedGraphGenerator(unsigned vertex_count,
                                 unsigned seed = ::gen::test::NewSeedVal())
      : super(vertex_count, UndirectedEdgeGenerator(), seed) { }
};

template<class TGenerator, typename T = unsigned>
class WeightedMultigraphGenerator :
  public ::gen::test::Generator<
  WeightedMultigraphGenerator<TGenerator, T>, Edge<T> > {
  typedef ::gen::test::Generator<
        WeightedMultigraphGenerator<TGenerator, T>, Edge<T> > Base;
public:
  WeightedMultigraphGenerator(unsigned vertex_count,
                              const TGenerator& edge_data_gen = TGenerator(),
                              unsigned seed = ::gen::test::NewSeedVal())
  : Base(seed),
    range_var_(0, vertex_count, seed),
    edge_data_gen_(edge_data_gen) { }

  inline Edge<T> operator()() {
    return Edge<T>(range_var_(), range_var_(), edge_data_gen_());
  }

private:
  ::gen::test::DiscreteUniformRangeRandomVariable<VertexId> range_var_;
  TGenerator edge_data_gen_;
};

}  // namespace test

}  // namespace gen

}  // namespace ksp

#endif /* GRAPH_GENERATOR_HPP_ */
