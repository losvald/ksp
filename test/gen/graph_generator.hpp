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
#include "random_utils.hpp"

namespace ksp {

namespace gen {

namespace test {

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
    // TODO GeneratorType doesn't work
public:
  SimpleWeightedGraphGenerator(unsigned vertex_count,
                               const TGenerator& edge_data_gen = TGenerator(),
                               unsigned seed = ::gen::test::NewSeedVal())
  : Base(seed, vertex_count * (vertex_count > 0 ? vertex_count - 1U : 0)),
    adj_vtx_cnt_(vertex_count > 0 ? vertex_count - 1U : 0),
    cur_ind_(vertex_count * adj_vtx_cnt_),
    edge_data_gen_(edge_data_gen) { }

  Edge<T> operator()() {
    if (cur_ind_ == 0)
      return NullEdge<T>();

    // swap cur_ind_ - 1 with a random index ind from range [0, cur_ind)
    std::size_t ind = ::gen::test::Random(cur_ind_--, Base::seed());
    IndexMap::iterator i = ind_map_.find(ind);
    std::size_t& ind_val = (i != ind_map_.end() ? i->second
                                                : ind_map_[ind] = ind);
    std::size_t ind_val_before_swap = ind_val;
    if ((i = ind_map_.find(cur_ind_)) != ind_map_.end()) {
      ind_val = i->second;
      ind_map_.erase(i);
    } else
      ind_val = cur_ind_;
    return ToEdge(ind_val_before_swap, edge_data_gen_());
  }

private:
  typedef std::map<std::size_t, std::size_t> IndexMap;

  inline Edge<T> ToEdge(std::size_t n, const T& edge_data) const {
    VertexId tail = n / adj_vtx_cnt_, head = n - tail * adj_vtx_cnt_;
    head += (tail <= head);
    return Edge<T>(tail, head, edge_data);
  }

  std::size_t adj_vtx_cnt_;
  std::size_t cur_ind_;
  IndexMap ind_map_;
  TGenerator edge_data_gen_;
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
