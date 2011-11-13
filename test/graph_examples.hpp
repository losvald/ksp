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

#ifndef GRAPH_EXAMPLES_HPP_
#define GRAPH_EXAMPLES_HPP_

#include <string>

#include "../src/graph_builder.hpp"

namespace ksp {

namespace test {

namespace example {

namespace graph {

namespace {
}

static GraphBuilder<char> LedaShortestPaths() {
  // http://www.leda-tutorial.org/en/unofficial/ch05s03s03.html
  char vmin = 'A', vmax = 'P';
  int n = vmax - vmin + 1;
  GraphBuilder<char> g(n);

  for (char v = vmin; v <= vmax; ++v)
    g.AddVertex(v);

  g.AddEdge('A', 'C', 7);
  g.AddEdge('A', 'D', 2);

  g.AddEdge('B', 'C', 1);
  g.AddEdge('B', 'D', 2);
  g.AddEdge('B', 'F', 3);
  g.AddEdge('B', 'G', 4);
  g.AddEdge('B', 'H', 2);

  g.AddEdge('C', 'B', 2);
  g.AddEdge('C', 'H', 2);
  g.AddEdge('C', 'I', 7);

  g.AddEdge('D', 'A', 3);
  g.AddEdge('D', 'B', 2);
  g.AddEdge('D', 'F', 2);

  g.AddEdge('E', 'A', 1);
  g.AddEdge('E', 'D', 2);

  g.AddEdge('F', 'G', 3);
  g.AddEdge('F', 'J', 2);

  g.AddEdge('G', 'F', 3);
  g.AddEdge('G', 'H', 3);
  g.AddEdge('G', 'J', 1);
  g.AddEdge('G', 'L', 1);
  g.AddEdge('G', 'M', 2);

  g.AddEdge('H', 'I', 2);
  g.AddEdge('H', 'K', 4);

  g.AddEdge('I', 'C', 7);
  g.AddEdge('I', 'K', 1);

  g.AddEdge('J', 'M', 1);
  g.AddEdge('J', 'N', 3);

  g.AddEdge('K', 'P', 3);

  g.AddEdge('L', 'K', 1);
  g.AddEdge('L', 'M', 1);
  g.AddEdge('L', 'O', 1);
  g.AddEdge('L', 'P', 4);

  g.AddEdge('M', 'N', 3);
  g.AddEdge('M', 'O', 2);

  g.AddEdge('O', 'N', 2);
  g.AddEdge('O', 'P', 1);

  g.AddEdge('P', 'N', 4);

  return g;
}

}  // namespace graph

}  // namespace example

}  // namespace test

}  // namespace ksp

#endif /* GRAPH_EXAMPLES_HPP_ */
