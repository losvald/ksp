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

#include <algorithm>
#include <string>
#include <sstream>

#include "../src/adjacency_list.hpp"
#include "../src/graph_builder.hpp"

#include "test.hpp"

#ifndef SKIP_TESTS

namespace ksp {

namespace test {

typedef GraphBuilder<int> MyGraph;
typedef MyGraph::EdgeType MyEdge;
typedef AdjacencyList<> MyAdjList;

struct MyEdgeLess {
  bool operator()(const MyEdge& x, const MyEdge& y) const {
      return x.tail < y.tail || (x.tail == y.tail &&
          (x.head < y.head || (x.head == y.head &&
              x.data < y.data)));
  }
};

struct MyEdgeEq {
  bool operator()(const MyEdge& x, const MyEdge& y) const {
    return x.tail == y.tail && x.head == y.head && x.data == y.data;
  }
};

void ExpectAdj(MyGraph g, const MyAdjList& a, MyGraph::VertexType v,
              MyEdge* exp_first, MyEdge* exp_last) {
  std::vector<MyEdge> act_edges(a.first(g.vertex_id(v)),
                                a.last(g.vertex_id(v)));
  std::vector<MyEdge> exp_edges(exp_first, exp_last);
  for (std::size_t i = 0; i < exp_edges.size(); ++i) {
    exp_edges[i].tail = g.vertex_id(exp_edges[i].tail);
    exp_edges[i].head = g.vertex_id(exp_edges[i].head);
  }
  if (!CheckSortedEq(exp_edges.begin(), exp_edges.end(),
                     act_edges.begin(), act_edges.end(),
                     MyEdgeLess(), MyEdgeEq())) {
    EXPECT_TRUE(false);
    std::cout << "  Actual:\n";
    for (std::size_t i = 0; i < exp_edges.size(); ++i)
      std::cout << act_edges[i]  << " ";
    std::cout << "\nExpected:\n";
    for (std::size_t i = 0; i < exp_edges.size(); ++i)
      std::cout << exp_edges[i] << " ";
    std::cout << "\n";
  }
}

TEST(adjacency_list, simple01) {
  MyGraph g;
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddEdge(1, 2, 3);
  g.AddEdge(3, 1, 4);
  g.AddEdge(3, 3, 5);

  MyAdjList a(g.edges_begin(), g.edges_end());

  MyEdge a1[] = {MyEdge(1, 2, 3)};
  ExpectAdj(g, a, 1, a1, a1 + 1);

  ExpectAdj(g, a, 2, a1, a1);

  MyEdge a3[] = {MyEdge(3, 1, 4), MyEdge(3, 3, 5)};
  ExpectAdj(g, a, 3, a3, a3 + 2);
}

TEST(adjacency_list, auto01) {
  int n = 100;
  MyGraph g;

  std::vector<int> vs(n);
  for (int i = 0; i < n; ++i)
    vs[i] = i;
  std::random_shuffle(vs.begin(), vs.end());
  for (int i = 0; i < n; ++i)
    g.AddVertex(i);

  for (int j = 0; j < n; ++j)
    for (int i = n - 1; i >= 0; --i)
      if (i >= j * j)
        g.AddEdge(i, j, 1);

  MyAdjList a(g.edges_begin(), g.edges_end());

  MyEdge a35[] = {MyEdge(35, 0, 1), MyEdge(35, 1, 1), MyEdge(35, 2, 1),
                  MyEdge(35, 3, 1), MyEdge(35, 4, 1), MyEdge(35, 5, 1)};
  ExpectAdj(g, a, 35, a35, a35 + 6);
}

}  // namespace test

}  // namespace ksp

#endif

