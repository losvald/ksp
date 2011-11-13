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

#include "../src/graph_builder.hpp"

#include "test.hpp"

#ifndef SKIP_TESTS

namespace ksp {

namespace test {

TEST(graph_builder, add01) {
  GraphBuilder<int> g;
  EXPECT_EQ(g.vertex_count(), 0);
  EXPECT_EQ(g.edge_count(), 0);

  g.AddVertex(1);
  EXPECT_EQ(g.vertex_count(), 1);
  g.AddVertex(2);
  EXPECT_EQ(g.vertex_count(), 2);
  g.AddVertex(3);
  EXPECT_EQ(g.vertex_count(), 3);

  g.AddEdge(1, 2, 3);
  EXPECT_EQ(g.edge_count(), 1);
  g.AddEdge(3, 1, 4);
  EXPECT_EQ(g.edge_count(), 2);
}

TEST(graph_builder, add02) {
  GraphBuilder<int> g;
  EXPECT_EQ(g.vertex_count(), 0);
  EXPECT_EQ(g.edge_count(), 0);

  g.AddVertex(1);
  EXPECT_EQ(g.vertex_count(), 1);
  g.AddVertex(2);
  EXPECT_EQ(g.vertex_count(), 2);
  g.AddVertex(3);
  EXPECT_EQ(g.vertex_count(), 3);


  EXPECT_NO_THROW(g.AddEdge(1, 2, 3));
  EXPECT_EQ(g.edge_count(), 1);

  EXPECT_THROW(g.AddEdge(5, 1, 4), NoSuchVertexException);
  EXPECT_THROW(g.AddEdge(1, 5, 6), NoSuchVertexException);
  EXPECT_EQ(g.edge_count(), 1);

  EXPECT_NO_THROW(g.AddEdge(3, 3, 3));
  EXPECT_EQ(g.edge_count(), 2);
}

}  // namespace test

}  // namespace ksp

#endif
