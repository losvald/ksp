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

#include <map>
#include <string>

#include "../src/edge_map.hpp"
#include "../src/graph_builder.hpp"

namespace ksp {

namespace test {

namespace example {

namespace graph {

typedef std::map<std::string, EdgeId> LedaReverseEdgeMap;

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

static std::map<std::string, EdgeId> LedaShortestPathsReverseEdgeMap() {
  std::map<std::string, EdgeId> ret;
  GraphBuilder<char> gb = LedaShortestPaths();
  EdgeId id = EdgeId();
  for (GraphBuilder<char>::EdgeMapType::const_iterator it = gb.edges_begin(),
      it_end = gb.edges_end(); it != it_end; ++it) {
    std::string s;
    s.push_back(gb.vertex(it->tail));
    s.push_back(gb.vertex(it->head));
    ret[s] = id++;
  }
  return ret;
}

static GraphBuilder<char> EppsteinFigure45ShortestPaths() {
  char v_out_min = 'a', v_out_max = 'a' + 12 - 1;
  int n = 5 + v_out_max - v_out_min + 1;

  GraphBuilder<char> g(n);
  g.AddVertex('p');
  g.AddVertex('q');
  g.AddVertex('r');
  g.AddVertex('s');
  g.AddVertex('t');
  for (char v = v_out_min; v <= v_out_max; ++v)
    g.AddVertex(v);

  g.AddEdge('p', 'q', 100);
  g.AddEdge('q', 'r', 100);
  g.AddEdge('s', 'r', 100);
  g.AddEdge('r', 't', 100);

  g.AddEdge('p', 'a',  1);  g.AddEdge('a', 't',  3*100);
  g.AddEdge('p', 'd',  6);  g.AddEdge('d', 't',  3*100);
  g.AddEdge('p', 'h', 12);  g.AddEdge('h', 't',  3*100);
  g.AddEdge('p', 'j', 14);  g.AddEdge('j', 't',  3*100);

  g.AddEdge('q', 'i', 13);  g.AddEdge('i', 't',  2*100);

  g.AddEdge('r', 'k', 17);  g.AddEdge('k', 't',  1*100);
  g.AddEdge('r', 'l', 19);  g.AddEdge('l', 't',  1*100);

  g.AddEdge('s', 'b',  3);  g.AddEdge('b', 't',  2*100);
  g.AddEdge('s', 'e',  7);  g.AddEdge('e', 't',  2*100);

  g.AddEdge('t', 'c',  4);  g.AddEdge('c', 't',  0*100);
  g.AddEdge('t', 'f',  8);  g.AddEdge('f', 't',  0*100);
  g.AddEdge('t', 'g', 10);  g.AddEdge('g', 't',  0*100);

  return g;
}

}  // namespace graph

}  // namespace example

}  // namespace test

}  // namespace ksp

#endif /* GRAPH_EXAMPLES_HPP_ */
