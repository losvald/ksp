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

#ifndef SHORTEST_PATH_TREE_EXAMPLES_HPP_
#define SHORTEST_PATH_TREE_EXAMPLES_HPP_

#include "../src/shortest_path_tree.hpp"

#include "test.hpp"
#include "graph_test_utils.hpp"
#include "graph_examples.hpp"

namespace ksp {

namespace test {

namespace example {

namespace spt {

namespace {

  template<typename V, typename T, class VCompare>
  void AddToSpt(const GraphBuilder<V, T, VCompare>& g, ShortestPathTree<T>* spt,
                T dist, V tail, V head, T edge_data) {
    spt->Add(dist, E(g, tail, head, edge_data));
  }


}  // namespace

static ShortestPathTree<> LedaShortestPaths() {
  GraphBuilder<char> g = graph::LedaShortestPaths();
  ShortestPathTree<> t(g.vertices_begin(), g.vertices_end(),
                       g.vertex_id('A'), 0x3f3f3f3f);
  AddToSpt<char, unsigned>(g, &t, 2, 'A', 'D', 2);

  AddToSpt<char, unsigned>(g, &t, 4, 'D', 'B', 2);
  AddToSpt<char, unsigned>(g, &t, 4, 'D', 'F', 2);

  AddToSpt<char, unsigned>(g, &t, 5, 'B', 'C', 1);
  AddToSpt<char, unsigned>(g, &t, 6, 'B', 'H', 2);

  AddToSpt<char, unsigned>(g, &t, 7, 'F', 'G', 3);
  AddToSpt<char, unsigned>(g, &t, 6, 'F', 'J', 2);

  AddToSpt<char, unsigned>(g, &t, 8, 'H', 'I', 2);

  AddToSpt<char, unsigned>(g, &t, 8, 'G', 'L', 1);


  AddToSpt<char, unsigned>(g, &t, 7, 'J', 'M', 1);
  AddToSpt<char, unsigned>(g, &t, 9, 'J', 'N', 3);

  AddToSpt<char, unsigned>(g, &t, 9, 'L', 'K', 1);

  AddToSpt<char, unsigned>(g, &t, 9, 'M', 'O', 2);

  AddToSpt<char, unsigned>(g, &t, 10, 'O', 'P', 1);

  return t;
}

}  // namespace spt

}  // namespace example

}  // namespace test

}  // namespace ksp

#endif /* SHORTEST_PATH_TREE_EXAMPLES_HPP_ */
