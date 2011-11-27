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

#include "../../src/edge_map.hpp"
#include "../../src/algo/dijkstra.hpp"
#include "../../src/algo/eppstein/path_graph.hpp"
#include "../../src/util/string_utils.hpp"

#include "../test.hpp"

#include "../graph_examples.hpp"
#include "../shortest_path_tree_examples.hpp"


#ifndef SKIP_TESTS

namespace ksp {

namespace test {

namespace {

const GraphBuilder<char> kGBLeda = example::graph::LedaShortestPaths();
const ShortestPathTree<> kSptLeda = example::spt::LedaShortestPaths();

EdgeMap<unsigned> LedaDeltas() {
  EdgeMap<unsigned> d;
  return d;
}

template<typename T, typename InputIterator, class TClosure, class TWeightDiff>
static void ComputeDeltas(InputIterator edges_begin, InputIterator edges_end,
                          const ShortestPathTree<T>& sp_tree,
                          EdgeMap<T>* deltas,
                          const TClosure& closure = std::plus<T>(),
                          const TWeightDiff& weight_diff = std::minus<T>()) {
  const VertexMap<T>& d = sp_tree.distance_map();
  deltas->Init(edges_begin, edges_end);
  for (EdgeId id = EdgeId(); edges_begin != edges_end; ++edges_begin) {
    (*deltas)[id++] = weight_diff(
        closure(edges_begin->data, d[edges_begin->tail]),
        d[edges_begin->head]);
  }
}

template<typename V, typename T, class VCompare, typename InputIterator>
std::string TopoToStr(InputIterator first, InputIterator last,
                      const GraphBuilder<V, T, VCompare>& g) {
  std::stringstream oss;
  oss << "[";
  while (first != last) {
    oss << g.vertex(*first);
    if (++first == last)
      oss << "]";
    else
      oss << ", ";
  }
  return oss.str();
}

example::graph::LedaReverseEdgeMap kLedaReverseEdgeMap =
    example::graph::LedaShortestPathsReverseEdgeMap();

GraphBuilder<char> kGBEppFig45 =
    example::graph::EppsteinFigure45ShortestPaths();

}  // namespace

TEST(eppstein, deltas_Leda) {
  EdgeMap<unsigned> exp_deltas(kGBLeda.edge_count());
  exp_deltas.at(kLedaReverseEdgeMap["AC"]) = 2;
  exp_deltas.at(kLedaReverseEdgeMap["AD"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["BC"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["BD"]) = 4;
  exp_deltas.at(kLedaReverseEdgeMap["BF"]) = 3;
  exp_deltas.at(kLedaReverseEdgeMap["BG"]) = 1;
  exp_deltas.at(kLedaReverseEdgeMap["BH"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["CB"]) = 3;
  exp_deltas.at(kLedaReverseEdgeMap["CH"]) = 1;
  exp_deltas.at(kLedaReverseEdgeMap["CI"]) = 4;

  exp_deltas.at(kLedaReverseEdgeMap["DA"]) = 5;
  exp_deltas.at(kLedaReverseEdgeMap["DB"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["DF"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["EA"]) = 0x3f3f3f3f + 1;
  exp_deltas.at(kLedaReverseEdgeMap["ED"]) = 0x3f3f3f3f;

  exp_deltas.at(kLedaReverseEdgeMap["FG"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["FJ"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["GF"]) = 6;
  exp_deltas.at(kLedaReverseEdgeMap["GH"]) = 4;
  exp_deltas.at(kLedaReverseEdgeMap["GJ"]) = 2;
  exp_deltas.at(kLedaReverseEdgeMap["GL"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["GM"]) = 2;


  exp_deltas.at(kLedaReverseEdgeMap["HI"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["HK"]) = 1;

  exp_deltas.at(kLedaReverseEdgeMap["IC"]) = 10;
  exp_deltas.at(kLedaReverseEdgeMap["IK"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["JM"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["JN"]) = 0;


  exp_deltas.at(kLedaReverseEdgeMap["KP"]) = 2;

  exp_deltas.at(kLedaReverseEdgeMap["LK"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["LM"]) = 2;
  exp_deltas.at(kLedaReverseEdgeMap["LO"]) = 0;
  exp_deltas.at(kLedaReverseEdgeMap["LP"]) = 2;


  exp_deltas.at(kLedaReverseEdgeMap["MN"]) = 1;
  exp_deltas.at(kLedaReverseEdgeMap["MO"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["ON"]) = 2;
  exp_deltas.at(kLedaReverseEdgeMap["OP"]) = 0;

  exp_deltas.at(kLedaReverseEdgeMap["PN"]) = 5;

  EdgeMap<unsigned> act_deltas;
  std::plus<unsigned> closure;
  std::minus<unsigned> weight_diff;
  std::equal_to<unsigned> weight_eq;
  ComputeDeltas(kGBLeda.edges_begin(), kGBLeda.edges_end(), kSptLeda,
                &act_deltas, closure, weight_diff);

  ASSERT_TRUE(CheckEq(exp_deltas.begin(), exp_deltas.end(),
                      act_deltas.begin(), act_deltas.end(), weight_eq)) <<
                      "Expected: " << ToString(exp_deltas.begin(),
                                               exp_deltas.end()) << std::endl <<
                      "Actual:   " << ToString(act_deltas.begin(),
                                               act_deltas.end()) << std::endl;
}

TEST(eppstein_path_graph, figure4a) {
  std::equal_to<unsigned> equals;
  std::minus<unsigned> weight_diff;

  AdjacencyList<> adj(kGBEppFig45.edges_begin(), kGBEppFig45.edges_end());
  EppsteinPathGraph<> p(adj, kGBEppFig45.vertex_id('t'),
                equals, weight_diff, 0x3f3f3f3f);

  for (VertexId i = 0; i < kGBEppFig45.vertex_count(); ++i)
    std::cout << kGBEppFig45.vertex(i) << " ";
  std::cout << "\n";
  std::cout << ToString(p.shortest_path_tree().distance_map().begin(),
                        p.shortest_path_tree().distance_map().end()) << "\n";

  std::cout << p << std::endl;

  typedef EppsteinPathGraph<>::Locator Loc;

  Loc t_root(p.locator(kGBEppFig45.vertex_id('t')));
  EXPECT_EQ(4, p.data(t_root).val);
  EXPECT_EQ(t_root, p.locator(kGBEppFig45.vertex_id('t')));
  EXPECT_EQ(1, p.heap_edge_count(t_root));
  Loc t_0(t_root); EXPECT_EQ(8 - 4, p.Descend(0, &t_0));
  EXPECT_EQ(1, p.heap_edge_count(t_0));
  Loc t_00(t_0); EXPECT_EQ(10 - 8, p.Descend(0, &t_00));
  EXPECT_EQ(0, p.heap_edge_count(t_00));

  Loc r_root(p.locator(kGBEppFig45.vertex_id('r')));
  EXPECT_EQ(4, p.data(r_root).val);
  EXPECT_NE(r_root, t_root);
  EXPECT_EQ(2, p.heap_edge_count(r_root));
  Loc r_0(r_root); EXPECT_EQ(17 - 4, p.Descend(0, &r_0));
  Loc r_00(r_0); EXPECT_EQ(19 - 17, p.Descend(0, &r_00));
  EXPECT_EQ(0, p.heap_edge_count(r_00));
  Loc r_1(r_root); EXPECT_EQ(8 - 4, p.Descend(1, &r_1));
  EXPECT_EQ(r_1, t_0);
  EXPECT_EQ(1, p.heap_edge_count(r_1));
  Loc r_10(r_1); EXPECT_EQ(10 - 8, p.Descend(0, &r_10));
  EXPECT_EQ(0, p.heap_edge_count(r_10));
  EXPECT_EQ(r_10, t_00);

  Loc s_root(p.locator(kGBEppFig45.vertex_id('s')));
  EXPECT_EQ(3, p.data(s_root).val);
  EXPECT_EQ(3, p.heap_edge_count(s_root));
  Loc s_0(s_root); EXPECT_EQ(17 - 3, p.Descend(0, &s_0));
  EXPECT_EQ(s_0, r_0);
  Loc s_1(s_root); EXPECT_EQ(4 - 3, p.Descend(1, &s_1));
  EXPECT_NE(s_1, r_root);
  EXPECT_EQ(1, p.heap_edge_count(s_1));
  Loc s_10(s_1); EXPECT_EQ(8 - 4, p.Descend(0, &s_10));
  EXPECT_EQ(s_10, t_0);
  Loc s_2(s_root); EXPECT_EQ(7 - 3, p.Descend(2, &s_2));
  EXPECT_EQ(0, p.heap_edge_count(s_2));

  Loc q_root(p.locator(kGBEppFig45.vertex_id('q')));
  EXPECT_EQ(4, p.data(q_root).val);
  EXPECT_NE(q_root, t_root);
  EXPECT_NE(q_root, r_root);
  EXPECT_NE(q_root, s_1);
  EXPECT_EQ(3, p.heap_edge_count(q_root));
  Loc q_0(q_root); EXPECT_EQ(17 - 4, p.Descend(0, &q_0));
  EXPECT_EQ(q_0, r_0);
  Loc q_1(q_root); EXPECT_EQ(13 - 4, p.Descend(1, &q_1));
  EXPECT_EQ(0, p.heap_edge_count(q_1));
  Loc q_2(q_root); EXPECT_EQ(8 - 4, p.Descend(2, &q_2));
  EXPECT_EQ(q_2, t_0);

  Loc p_root(p.locator(kGBEppFig45.vertex_id('p')));
  EXPECT_EQ(1, p.data(p_root).val);
  EXPECT_EQ(3, p.heap_edge_count(p_root));
  Loc p_0(p_root); EXPECT_EQ(4 - 1, p.Descend(0, &p_0));
  Loc p_1(p_root); EXPECT_EQ(13 - 1, p.Descend(1, &p_1));
  Loc p_2(p_root); EXPECT_EQ(6 - 1, p.Descend(2, &p_2));
  EXPECT_EQ(2, p.heap_edge_count(p_2));
  Loc p_20(p_2); EXPECT_EQ(12 - 6, p.Descend(0, &p_20));
  EXPECT_EQ(0, p.heap_edge_count(p_20));
  Loc p_21(p_2); EXPECT_EQ(14 - 6, p.Descend(1, &p_21));
  EXPECT_EQ(0, p.heap_edge_count(p_21));


  Loc t_root_cross(t_root); EXPECT_EQ(4, p.Cross(&t_root_cross));
  EXPECT_EQ(t_root_cross, t_root);
  EXPECT_EQ(4, p.Cross(&t_root_cross));
  EXPECT_EQ(t_root_cross, t_root);

  // auto generated by
  // grep -E 'Loc ?' | sed -re 's/\(.*//' -e 's/[^c]+c (.*)/&_cross(\1)/'
  // -e 's/Loc ([^(]+)(.*)/Loc \1\2; EXPECT_EQ(4, p.Cross(\&\1));/'
  Loc t_0_cross(t_0); EXPECT_EQ(4, p.Cross(&t_0_cross));
  Loc t_00_cross(t_00); EXPECT_EQ(4, p.Cross(&t_00_cross));
  Loc r_root_cross(r_root); EXPECT_EQ(4, p.Cross(&r_root_cross));
  Loc r_0_cross(r_0); EXPECT_EQ(4, p.Cross(&r_0_cross));
  Loc r_00_cross(r_00); EXPECT_EQ(4, p.Cross(&r_00_cross));
  Loc r_1_cross(r_1); EXPECT_EQ(4, p.Cross(&r_1_cross));
  Loc r_10_cross(r_10); EXPECT_EQ(4, p.Cross(&r_10_cross));
  Loc s_root_cross(s_root); EXPECT_EQ(4, p.Cross(&s_root_cross));
  Loc s_0_cross(s_0); EXPECT_EQ(4, p.Cross(&s_0_cross));
  Loc s_1_cross(s_1); EXPECT_EQ(4, p.Cross(&s_1_cross));
  Loc s_10_cross(s_10); EXPECT_EQ(4, p.Cross(&s_10_cross));
  Loc s_2_cross(s_2); EXPECT_EQ(4, p.Cross(&s_2_cross));
  Loc q_root_cross(q_root); EXPECT_EQ(4, p.Cross(&q_root_cross));
  Loc q_0_cross(q_0); EXPECT_EQ(4, p.Cross(&q_0_cross));
  Loc q_1_cross(q_1); EXPECT_EQ(4, p.Cross(&q_1_cross));
  Loc q_2_cross(q_2); EXPECT_EQ(4, p.Cross(&q_2_cross));
  Loc p_root_cross(p_root); EXPECT_EQ(4, p.Cross(&p_root_cross));
  Loc p_0_cross(p_0); EXPECT_EQ(4, p.Cross(&p_0_cross));
  Loc p_1_cross(p_1); EXPECT_EQ(4, p.Cross(&p_1_cross));
  Loc p_2_cross(p_2); EXPECT_EQ(4, p.Cross(&p_2_cross));
  Loc p_20_cross(p_20); EXPECT_EQ(4, p.Cross(&p_20_cross));
  Loc p_21_cross(p_21); EXPECT_EQ(4, p.Cross(&p_21_cross));
}

}  // namespace test

}  // namespace ksp

#endif
