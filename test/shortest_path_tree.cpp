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

#include <functional>

#include "../src/shortest_path_tree.hpp"
#include "../src/util/string_utils.hpp"

#include "test.hpp"

#include "edge_test_utils.hpp"
#include "graph_examples.hpp"
#include "graph_test_utils.hpp"
#include "shortest_path_tree_examples.hpp"
#include "spt_test_utils.hpp"

#ifndef SKIP_TESTS

namespace ksp {

namespace test {

namespace {

const GraphBuilder<char> kGBLeda = example::graph::LedaShortestPaths();
const ShortestPathTree<> kSptLeda = example::spt::LedaShortestPaths();


}

TEST(shortest_path_tree, vertex_count_GLeda) {
  EXPECT_EQ(15, kSptLeda.vertex_count());
}

TEST(shortest_path_tree, vertex_count_trivial) {
  GraphBuilder<int> g;
  g.AddVertex(2);
  ShortestPathTree<> spt(g.vertices_begin(), g.vertices_end(), 2, 0x3f3f3f3f);
  EXPECT_EQ(1, spt.vertex_count());
}

TEST(shortest_path_tree, vertices_SptLeda) {
  std::vector<VertexId> act(kSptLeda.vertices_begin(), kSptLeda.vertices_end());
  EXPECT_EQ(15, act.size());

  std::vector<VertexId> exp;
  for (char v = 'A'; v <= 'P'; ++v)
    if (v != 'E') {
      VertexId v_id = kGBLeda.vertex_id(v);
      kSptLeda.is_reachable(v_id);
      exp.push_back(v_id);
    }
  std::sort(act.begin(), act.end());
  EXPECT_TRUE(CheckEq(exp.begin(), exp.end(), act.begin(), act.end(),
              std::equal_to<VertexId>()));
}

TEST(shortest_path_tree, edges_SptLeda) {
  EdgeEqual<> eq;

  std::vector<Edge<> > exp_edges;
  exp_edges.push_back(E(kGBLeda, 'A', 'D', 2U));
  exp_edges.push_back(E(kGBLeda, 'D', 'B', 2U));
  exp_edges.push_back(E(kGBLeda, 'D', 'F', 2U));
  exp_edges.push_back(E(kGBLeda, 'B', 'C', 1U));
  exp_edges.push_back(E(kGBLeda, 'B', 'H', 2U));
  exp_edges.push_back(E(kGBLeda, 'F', 'G', 3U));
  exp_edges.push_back(E(kGBLeda, 'F', 'J', 2U));
  exp_edges.push_back(E(kGBLeda, 'H', 'I', 2U));
  exp_edges.push_back(E(kGBLeda, 'G', 'L', 1U));
  exp_edges.push_back(E(kGBLeda, 'J', 'M', 1U));
  exp_edges.push_back(E(kGBLeda, 'J', 'N', 3U));
  exp_edges.push_back(E(kGBLeda, 'L', 'K', 1U));
  exp_edges.push_back(E(kGBLeda, 'M', 'O', 2U));
  exp_edges.push_back(E(kGBLeda, 'O', 'P', 1U));

  EXPECT_TRUE(CheckEq(exp_edges.begin(), exp_edges.end(),
                      kSptLeda.edges_begin(), kSptLeda.edges_end(), eq)) <<
                      "Actual edges: " <<
                      ToString(kSptLeda.edges_begin(), kSptLeda.edges_end()) <<
                      std::endl;
}

TEST(shortest_path_tree, distance_SptLeda) {
  EXPECT_EQ(0, kSptLeda.distance(kGBLeda.vertex_id('A')));
  EXPECT_EQ(7, kSptLeda.distance(kGBLeda.vertex_id('G')));
//  EXPECT_THROW(kSptLeda.distance(kGBLeda.vertex_id('E')),
//               VertexUnreachableException);
  EXPECT_EQ(kSptLeda.distance(kGBLeda.vertex_id('E')), 0x3f3f3f3f);
  EXPECT_EQ(10, kSptLeda.distance(kGBLeda.vertex_id('P')));
}

TEST(shortest_path_tree, path_SptLeda_K) {
  EdgeEqual<> eq;
  ShortestPathTreeIterator<>
      path_rbegin = kSptLeda.path_rbegin(kGBLeda.vertex_id('K')),
      path_rend = kSptLeda.path_rend(kGBLeda.vertex_id('K')),
      path_it = path_rbegin;

  EXPECT_TRUE(path_it == path_rbegin);
  EXPECT_TRUE(path_it != path_rend);

  EXPECT_FALSE(eq(E(kGBLeda, 'K', 'L', 1U), *path_it)) << EStr(kGBLeda, *path_it);

  EXPECT_TRUE(eq(E(kGBLeda, 'L', 'K', 1U), *path_it)) << EStr(kGBLeda, *path_it);
  ++path_it;
  EXPECT_TRUE(eq(E(kGBLeda, 'G', 'L', 1U), *path_it)) << EStr(kGBLeda, *path_it);
  ++path_it;
  ++path_it;
  ++path_it;
  EXPECT_TRUE(eq(E(kGBLeda, 'A', 'D', 2U), *path_it)) << EStr(kGBLeda, *path_it);
  EXPECT_TRUE(path_it != path_rend);
  ++path_it;
  EXPECT_TRUE(path_it == path_rend);
}

TEST(shortest_path_tree, path_SptLeda_E) {
  EXPECT_THROW(kSptLeda.path_rbegin(kGBLeda.vertex_id('E')),
               VertexUnreachableException);
  EXPECT_THROW(kSptLeda.path_rend(kGBLeda.vertex_id('E')),
               VertexUnreachableException);
}

TEST(shortest_path_tree, path_SptLeda_A) {
  ShortestPathTreeIterator<>
    path_it = kSptLeda.path_rbegin(kGBLeda.vertex_id('A')),
    path_rend = kSptLeda.path_rend(kGBLeda.vertex_id('A'));
  EXPECT_TRUE(path_it == path_rend);
}

TEST(shortest_path_tree_utils, IsValidSpt_SptLeda) {
  std::less<unsigned> compare; std::plus<unsigned> closure;
  EXPECT_TRUE(IsValidSpt(kSptLeda, compare, closure));
}

TEST(shortest_path_tree_utils, IsValidSpt_SptLeda_bad_dist) {
  std::less<unsigned> compare; std::plus<unsigned> closure;
  ShortestPathTree<> spt = kSptLeda;

  spt.Update(9, E(kGBLeda, 'M', 'O', 2U));
  EXPECT_TRUE(IsValidSpt(spt, compare, closure));

  spt.Update(8, E(kGBLeda, 'M', 'O', 1U));
  ASSERT_EQ(8, spt.distance(kGBLeda.vertex_id('O')));
  ASSERT_EQ(1U, spt.path_rbegin(kGBLeda.vertex_id('O'))->data);
  EXPECT_FALSE(IsValidSpt(spt, compare, closure));

  spt.Update(8, E(kGBLeda, 'O', 'P', 0U));
  ASSERT_EQ(8, spt.distance(kGBLeda.vertex_id('P')));
  ASSERT_EQ(0U, spt.path_rbegin(kGBLeda.vertex_id('P'))->data);
  EXPECT_TRUE(IsValidSpt(spt, compare, closure));
}

TEST(shortest_path_tree_utils, IsValidSpt_SptLeda_bad_edge) {
  std::less<unsigned> compare; std::plus<unsigned> closure;
  ShortestPathTree<> spt = kSptLeda;

  spt.Add(5, E(kGBLeda, 'A', 'E', 5U));
  EXPECT_TRUE(IsValidSpt(spt, compare, closure));

  spt.Update(4, E(kGBLeda, 'A', 'E', 3U));
  EXPECT_FALSE(IsValidSpt(spt, compare, closure));
}

TEST(shortest_path_tree_utils, SptEquiv_SptLeda_false_01) {
  std::less<unsigned> compare; std::plus<unsigned> closure;
  SptEquiv<> spt_equiv;

  ShortestPathTree<> spt = kSptLeda;
  spt.Update(9, E(kGBLeda, 'M', 'O', 2U));
  spt.Update(8, E(kGBLeda, 'M', 'O', 1U));
  spt.Update(8, E(kGBLeda, 'O', 'P', 0U));
  ASSERT_TRUE(IsValidSpt(spt, compare, closure));
  EXPECT_FALSE(spt_equiv(spt, kSptLeda));
}

TEST(shortest_path_tree_utils, SptEquiv_SptLeda_false_02) {
  std::less<unsigned> compare; std::plus<unsigned> closure;
  SptEquiv<> spt_equiv;

  ShortestPathTree<> spt = kSptLeda;
  spt.Add(1, E(kGBLeda, 'A', 'E', 1U));
  spt.Add(2, E(kGBLeda, 'E', 'D', 1U));
  ASSERT_TRUE(IsValidSpt(spt, compare, closure));
  EXPECT_FALSE(spt_equiv(spt, kSptLeda));
}

TEST(shortest_path_tree_utils, SptEquiv_SptLeda_true) {
  std::less<unsigned> compare; std::plus<unsigned> closure;
  SptEquiv<> spt_equiv;

  ShortestPathTree<> spt1 = kSptLeda, spt2 = kSptLeda;
  spt1.Update(5 - 1, E(kGBLeda, 'A', 'C', 5U - 1U));
  spt2.Update(5 - 1, E(kGBLeda, 'B', 'C', 1U - 1U));
  ASSERT_TRUE(IsValidSpt(spt1, compare, closure));
  ASSERT_TRUE(IsValidSpt(spt2, compare, closure));
  EXPECT_TRUE(spt_equiv(spt1, spt2));
}

}  // namespace test

}  // namespace ksp

#endif
