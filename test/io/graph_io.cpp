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

#include "../../src/graph_builder.hpp"
#include "../../src/io/graph_io.hpp"

#include "../test.hpp"

#ifndef SKIP_TESTS

namespace ksp {

namespace test {

namespace {

typedef GraphBuilder<int> MyGraphBuilder;
typedef MyGraphBuilder::EdgeType MyEdge;

struct MyEdgeLess {
  bool operator()(const MyEdge& x, const MyEdge& y) const {
      return x.tail < y.tail || (x.tail == y.tail &&
          (x.head < y.head || (x.head == y.head &&
              x.data < y.data)));
  }
};
struct MyEdgeEq {
  bool operator()(const MyEdge& x, const MyEdge& y) const {
    return x.tail == y.tail && x.head == y.head &&
        x.data == y.data;
  }
};

std::string ToUniqueString(const MyGraphBuilder& g) {
  typedef std::vector<std::string> Lines;
  Lines lines;
  for (MyGraphBuilder::EdgeMapType::const_iterator e = g.edges_begin();
        e != g.edges_end(); ++e) {
      std::ostringstream line;
      line << g.vertex(e->tail) << " " << g.vertex(e->head) << " " <<
          e->data << std::endl;
      lines.push_back(line.str());
  }
  std::sort(lines.begin(), lines.end());
  std::string ret;
  for (Lines::iterator l = lines.begin(); l != lines.end(); ++l) {
    ret += *l;
  }
  return ret;
}

void ExpectEqual(const MyGraphBuilder& g, const std::string& s) {
  MyGraphBuilder g2;
  std::istringstream iss2(s);
  ReadGraph(iss2, &g2);
  EXPECT_EQ(ToUniqueString(g), ToUniqueString(g2));
}

}

TEST(graph_io, eq1) {
  MyGraphBuilder g;
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddEdge(1, 2, 3);
  g.AddEdge(3, 1, 4);

  ExpectEqual(g,
              "3 2\n"
              "1\n"
              "2\n"
              "3\n"
              "1 2 3\n"
              "3 1 4\n");
  ExpectEqual(g,
              "3 2\n"
              "2\n"
              "3\n"
              "1\n"
              "3 1 4\n"
              "1 2 3\n");
}

TEST(graph_io, eq2) {
  MyGraphBuilder g;
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddEdge(1, 2, 3);
  g.AddEdge(3, 1, 4);

  ExpectEqual(g,
              "3 2\n"
              "1\n"
              "2\n"
              "3\n"
              "1 2 3\n"
              "3 1 4\n");
  ExpectEqual(g,
              "3 2\n"
              "2\n"
              "3\n"
              "1\n"
              "3 1 4\n"
              "1 2 3\n");
}

}  // namespace test

}  // namespace ksp

#endif
