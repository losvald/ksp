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

#include <ctime>
#include <limits>

#include "../../src/adjacency_list.hpp"
#include "../../src/algo/dijkstra.hpp"
#include "../../src/util/string_utils.hpp"

#include "../test.hpp"
#include "../edge_test_utils.hpp"
#include "../gen/graph_generator.hpp"
#include "../graph_examples.hpp"
#include "../shortest_path_tree_examples.hpp"
#include "../spt_test_utils.hpp"

#include "dijkstra_multimap.hpp"

#ifndef SKIP_TESTS

namespace ksp {

namespace test {

namespace {

const GraphBuilder<char> kGBLeda = example::graph::LedaShortestPaths();
const ShortestPathTree<> kSptLeda = example::spt::LedaShortestPaths();

struct LedaDijkstraVisitor : public DijkstraVisitorAdapter<unsigned,
const GraphBuilder<char> > {
//  void InitVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
//                  const Graph<char>* g) const {
//    std::cerr << "InitVertex(" << g->vertex(v_id) << ")\n";
//  }

  void ExamineVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
                     const GraphBuilder<char>* g) const {
    std::cerr << "ExamineVertex(" << g->vertex(v_id) << ")\n";
  }

//  void ExamineEdge(const Edge<>& e, const ShortestPathTree<>& sp_tree,
//                   const Graph<char>* g) const {
//    std::cerr << "ExamineEdge(" << EStr(*g, e) << ")\n";
//  }

  void DiscoverVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
                      const GraphBuilder<char>* g) const {
    std::cerr << "DiscoverVertex(" << g->vertex(v_id) << ")\n";
  }

  void EdgeRelaxed(const Edge<>& e, const ShortestPathTree<>& sp_tree,
                   const GraphBuilder<char>* g) const {
    std::cerr << "EdgeRelaxed(" << EStr(*g, e) << ")\n";
  }

  void EdgeNotRelaxed(const Edge<>& e, const ShortestPathTree<>& sp_tree,
                      const GraphBuilder<char>* g) const {
    std::cerr << "EdgeNotRelaxed(" << EStr(*g, e) << ")\n";
  }

  void FinishVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
                    const GraphBuilder<char>* g) const {
    std::cerr << "FinishVertex(" << g->vertex(v_id) << ")\n";
  }
}
// kDebugVisLeda
;

} // namespace

TEST(dijkstra, Leda) {
  std::less<unsigned> compare;
  std::plus<unsigned> closure;
  SptEquiv<> spt_equiv;
  ShortestPathTree<> act_spt;

  typedef const GraphBuilder<char> MyContext;
  DijkstraShortestPaths(
      kGBLeda.edges_begin(), kGBLeda.edges_end(),
      kGBLeda.vertex_id('A'), &act_spt
      , std::numeric_limits<unsigned>::max()
      , compare, closure
// , kDebugVisLeda
//   , &kGBLeda
// , static_cast<MyContext*>(NULL)
   );
  EXPECT_TRUE(IsValidSpt(act_spt, compare, closure));
  AdjacencyList<> adj(act_spt.edges_begin(), act_spt.edges_end());

  EXPECT_TRUE(spt_equiv(kSptLeda, act_spt));
}

namespace {

typedef gen::test::SimpleWeightedGraphGenerator<
    gen::test::UniformEdgeWeightGenerator<> > SimpleGraphGen;
typedef gen::test::WeightedMultigraphGenerator<
    gen::test::UniformEdgeWeightGenerator<> > MultigraphGen;
typedef gen::test::UniformEdgeWeightGenerator<> Uewg;

}  // namespace


TEST(gen_g, foo) {
  const unsigned seed = 2U;
  Uewg weight_gen(1, 10, seed);
  SimpleGraphGen graph_gen(10, weight_gen, seed);
  AdjacencyList<> adj(graph_gen.begin(), graph_gen.begin().Skip(5));
  for (int i = 0; i < 5; ++i)
    std::cout << EToStr(graph_gen()) << std::endl;
  std::cout << adj << std::endl;
}

struct DebugDijkstraVisitor : public DijkstraVisitorAdapter<unsigned, void> {
//  void InitVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
//                  const Graph<char>* g) const {
//    std::cerr << "InitVertex(" << g->vertex(v_id) << ")\n";
//  }

  void ExamineVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
                     void* c) const {
    std::cerr << "ExamineVertex(" << v_id << ")\n";
  }

  void ExamineEdge(const Edge<>& e, const ShortestPathTree<>& sp_tree,
                   void* c) const {
    std::cerr << "ExamineEdge(" << e << ")\n";
  }

  void DiscoverVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
                      void* c) const {
    std::cerr << "DiscoverVertex(" << v_id << ")\n";
  }

  void EdgeRelaxed(const Edge<>& e, const ShortestPathTree<>& sp_tree,
                   void* c) const {
    std::cerr << "EdgeRelaxed(" << e << ")\n";
  }

  void EdgeNotRelaxed(const Edge<>& e, const ShortestPathTree<>& sp_tree,
                      void* c) const {
    std::cerr << "EdgeNotRelaxed(" << e << ")\n";
  }

  void FinishVertex(VertexId v_id, const ShortestPathTree<>& sp_tree,
                    void* c) const {
    std::cerr << "FinishVertex(" << v_id << ")\n";
  }
} kDebugVisitor;

//TEST(dijkstra, gen_iter) {
//  using namespace std;
//  double clock0;
//  const unsigned seed = 42U;
//  std::less<unsigned> compare;
//  std::plus<unsigned> closure;
//  unsigned dist_max = std::numeric_limits<unsigned>::max();
//  SptEquiv<> spt_equiv;
//  ShortestPathTree<> exp_spt, act_spt;
//
//  Uewg weight_gen(1, 100, seed);
//  SimpleGraphGen graph_gen(1000, weight_gen, seed);
//  SimpleGraphGen::Iter it = graph_gen.begin(), it2 = graph_gen.end();
//
//
//  AdjacencyList<> adj(graph_gen.begin(), graph_gen.begin().Skip(1000));
//}

TEST(dijkstra, random_small_01) {
//  const unsigned seed = 2U;
  std::less<unsigned> compare;
  std::plus<unsigned> closure;
  unsigned dist_max = std::numeric_limits<unsigned>::max();
  SptEquiv<> spt_equiv;
  ShortestPathTree<> exp_spt, act_spt;

  for (unsigned seed = 1; seed < 2000 /*seed < 500*/; ++seed) {
    Uewg weight_gen(1, 100, seed);
    SimpleGraphGen graph_gen(10, weight_gen, seed);
    AdjacencyList<> adj(graph_gen.begin(), graph_gen.end());


    DijkstraShortestPathsMM(adj.edges_begin(), adj.edges_end(), 0, &exp_spt,
                          dist_max, compare, closure);
    DijkstraShortestPaths(adj.edges_begin(), adj.edges_end(), 0, &act_spt,
                           dist_max, compare, closure);

    if (!spt_equiv(exp_spt, act_spt)) {
      gDebugTT ^= true;
      std::cerr << "\n============ EXPECTED BEGIN ==========\n";
      DijkstraShortestPathsMM(adj.edges_begin(), adj.edges_end(), 0, &act_spt,
                            dist_max, compare, closure,
                            kDebugVisitor, static_cast<void*>(NULL));
      std::cerr << "\n============ ACTUAL   BEGIN ==========\n";
      DijkstraShortestPaths(adj.edges_begin(), adj.edges_end(), 0, &act_spt,
                             dist_max, compare, closure,
                             kDebugVisitor, static_cast<void*>(NULL));
      gDebugTT ^= true;
      ASSERT_TRUE(spt_equiv(exp_spt, act_spt)) << adj << std::endl <<
          "Seed = " << seed << "\n" <<
          "Expected: " << ToString(exp_spt.distance_map().begin(),
                                   exp_spt.distance_map().end()) << "\n" <<
          "Actual:   " << ToString(act_spt.distance_map().begin(),
                                   act_spt.distance_map().end()) << "\n";
    }
  }
}

TEST(dijkstra, random_big_sparse) {
  using namespace std;
  double clock0;
  std::less<unsigned> compare;
  std::plus<unsigned> closure;
  SptEquiv<> spt_equiv;
  unsigned dist_max = std::numeric_limits<unsigned>::max();
  ShortestPathTree<> exp_spt, act_spt;

  const unsigned n = 200000, m = 3000000, seed = 5U;
  Uewg weight_gen(1, 1000, seed);
  MultigraphGen graph_gen(n, weight_gen, seed);

  clock0 = clock();
  std::vector<Edge<> > edges(m, NullEdge<unsigned>());
  for (std::size_t i = 0; i < m; ++i)
    edges[i] = graph_gen();
  cout << "Gen + adj: " << (clock() - clock0) / CLOCKS_PER_SEC << endl;

  clock0 = clock();
  DijkstraShortestPathsMM(edges.begin(), edges.end(), 0, &exp_spt,
                        dist_max, compare, closure);
  cout << "Multimap: " << (clock() - clock0) / CLOCKS_PER_SEC << endl;

  clock0 = clock();
  DijkstraShortestPaths(edges.begin(), edges.end(), 0, &act_spt,
                           dist_max, compare, closure);
  cout << "Interval: " << (clock() - clock0) / CLOCKS_PER_SEC << endl;
  ASSERT_TRUE(spt_equiv(exp_spt, act_spt));
}

TEST(dijkstra, random_big_dense) {
  const int k = 2;
  using namespace std;
  double clock0;
  std::less<unsigned> compare;
  std::plus<unsigned> closure;
  SptEquiv<> spt_equiv;
  unsigned dist_max = std::numeric_limits<unsigned>::max();
  ShortestPathTree<> exp_spt, act_spt;

  const unsigned n = k*5000, m = k*k*2000000, seed = 5U;
  Uewg weight_gen(1, 1000, seed);
  MultigraphGen graph_gen(n, weight_gen, seed);
//  ::gen::test::DiscreteRandomVariable<unsigned> var;
//  for (unsigned w = 0; w < 10; ++w)
//    var.Add(w, (11 - w) * (w + 1));
//    var.Add(2, 1); var.Add(3, 1); var.Add(7, 1); var.Add(11, 1);

  clock0 = clock();
  std::vector<Edge<> > edges(m, NullEdge<unsigned>());
  for (std::size_t i = 0; i < m; ++i) {
    edges[i] = graph_gen();
//    edges[i].data = var();
  }
  cout << "gen + vec: " << (clock() - clock0) / CLOCKS_PER_SEC << endl;

  clock0 = clock();
  DijkstraShortestPathsMM(edges.begin(), edges.end(), 0, &exp_spt,
                        dist_max, compare, closure);
  cout << "Multimap: " << (clock() - clock0) / CLOCKS_PER_SEC << endl;

  clock0 = clock();
  DijkstraShortestPaths(edges.begin(), edges.end(), 0, &act_spt,
                         dist_max, compare, closure);
  cout << "Interval: " << (clock() - clock0) / CLOCKS_PER_SEC << endl;
  ASSERT_TRUE(spt_equiv(exp_spt, act_spt));
}

} // namespace test

} // namespace ksp


#endif
