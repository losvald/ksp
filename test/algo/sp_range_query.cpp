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

#include "../../src/util/string_utils.hpp"

#include "../../src/algo/sp_range_query.hpp"

#include "../test.hpp"

#include "../graph_examples.hpp"
#include "../shortest_path_tree_examples.hpp"

#ifndef SKIP_TESTS

namespace ksp {

namespace test {

namespace {

template<typename V, typename T, class VCompare, class TCompare>
std::string SkipVerticesToString(
    const ShortestPathRangeQuery<T, TCompare>& sp_rq,
    VertexId v,
    const GraphBuilder<V, T, VCompare>& g) {
  std::stringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < sp_rq.skip_vertices(v).size(); ++i) {
    if (i) oss << ", ";
    oss << g.vertex(sp_rq.skip_vertices(v)[i]);
  }
  oss << "]";
  return oss.str();
}

const GraphBuilder<char> kGBLeda = example::graph::LedaShortestPaths();
const ShortestPathTree<> kSptLeda = example::spt::LedaShortestPaths();

}  // namespace

TEST(sp_range_query, Leda) {
  ShortestPathRangeQuery<> sp_rq(kSptLeda);

  for (VertexId v = 0; v < kGBLeda.vertex_count(); ++v) {
    std::cout << "skip(" << kGBLeda.vertex(v) << ") = " <<
        SkipVerticesToString(sp_rq, v, kGBLeda) << std::endl;
  }

  EXPECT_EQ(2, sp_rq(kGBLeda.vertex_id('A'), kGBLeda.vertex_id('D')));
  EXPECT_EQ(9, sp_rq(kGBLeda.vertex_id('A'), kGBLeda.vertex_id('K')));

  EXPECT_EQ(0, sp_rq(kGBLeda.vertex_id('G'), kGBLeda.vertex_id('G')));

  EXPECT_EQ(8, sp_rq(kGBLeda.vertex_id('D'), kGBLeda.vertex_id('P')));
  EXPECT_EQ(3, sp_rq(kGBLeda.vertex_id('J'), kGBLeda.vertex_id('N')));
  EXPECT_EQ(3, sp_rq(kGBLeda.vertex_id('D'), kGBLeda.vertex_id('C')));

  EXPECT_TRUE(sp_rq.is_reachable(kGBLeda.vertex_id('O'),
                                 kGBLeda.vertex_id('A')));
  EXPECT_FALSE(sp_rq.is_reachable(kGBLeda.vertex_id('E'),
                                  kGBLeda.vertex_id('A')));

  EXPECT_TRUE(sp_rq.is_reachable(kGBLeda.vertex_id('L'),
                                 kGBLeda.vertex_id('F')));
  EXPECT_TRUE(sp_rq.is_reachable(kGBLeda.vertex_id('N'),
                                 kGBLeda.vertex_id('D')));
  EXPECT_TRUE(sp_rq.is_reachable(kGBLeda.vertex_id('P'),
                                 kGBLeda.vertex_id('J')));

  EXPECT_TRUE(sp_rq.is_reachable(kGBLeda.vertex_id('C'),
                                 kGBLeda.vertex_id('C')));

  EXPECT_FALSE(sp_rq.is_reachable(kGBLeda.vertex_id('H'),
                                  kGBLeda.vertex_id('C')));
  EXPECT_FALSE(sp_rq.is_reachable(kGBLeda.vertex_id('N'),
                                  kGBLeda.vertex_id('B')));
  EXPECT_FALSE(sp_rq.is_reachable(kGBLeda.vertex_id('F'),
                                  kGBLeda.vertex_id('E')));
}

}  // namespace test

}  // namespace ksp

#endif
