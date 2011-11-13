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

#include "../src/util/string_utils.hpp"

#include "test.hpp"

#include "gen/discrete_random_variable.hpp"
#include "gen/gen_iter.hpp"
#include "gen/generator.hpp"
#include "gen/edge_generator.hpp"
#include "gen/graph_generator.hpp"

#include "edge_test_utils.hpp"
#include "graph_test_utils.hpp"

namespace ksp {

namespace gen {

namespace test {

namespace {

struct FibonacciGenerator
: public ::gen::test::Generator<FibonacciGenerator, int> {

  FibonacciGenerator(unsigned seed = ::gen::test::NewSeedVal()) :
    GeneratorType(seed),
    last2(-1), last1(1) { }

  int operator()() {
    int cur = last1 + last2;
    last2 = last1;
    last1 = cur;
    return cur;
  }

private:
  int last2, last1;
};

}  // namespace

TEST(gen_iter, simple01) {
  EXPECT_EQ(*FibonacciGenerator().begin(), *FibonacciGenerator::Iter());
}

TEST(gen_iter, simple02) {
  int fib_arr[] = {0, 1, 1, 2, 3};
  FibonacciGenerator fib_gen;

  EXPECT_TRUE(CheckEq(fib_gen.begin(), fib_gen.begin().Skip(5),
                      fib_arr, fib_arr + 5,
                      std::equal_to<int>()));
  EXPECT_FALSE(CheckEq(fib_gen.begin(), fib_gen.begin().Skip(5),
                       fib_arr, fib_arr + 4,
                       std::equal_to<int>()));
}

TEST(gen_iter, diff_minus) {
  FibonacciGenerator fib_gen;
  EXPECT_EQ(2, ++ ++ fib_gen.begin() - fib_gen.begin());
  EXPECT_EQ(9, fib_gen.begin().Skip(9) - fib_gen.begin());
}

struct MyGen : ::gen::test::GeneratorIteratorProvider<MyGen, int> {
  MyGen() : cnt_(0) { }

  int operator()() {
    if (cnt_++ > 0)
      throw cnt_;
    return cnt_;
  }

  int cnt_;
};

TEST(gen_iter, diff_distance_skips) {
  MyGen my_gen;
  ASSERT_NO_THROW(my_gen.begin());
  ASSERT_ANY_THROW(++my_gen.begin());

  ASSERT_EQ(5, my_gen.begin().Skip(5) - my_gen.begin());
  EXPECT_EQ(5, std::distance(my_gen.begin(), my_gen.begin().Skip(5)));
}


struct MyRandomGenerator
: public ::gen::test::Generator<FibonacciGenerator, int> {
  MyRandomGenerator(unsigned seed) : GeneratorType(seed) { }
  int operator()() {
    return ::gen::test::Random(1, 100000, seed());
  }
};

TEST(generator, initial_seed_copy) {
  const int seed = 5323;
  MyRandomGenerator gen1(seed), gen3(seed);
  gen1(); gen1(); gen1(); gen1();
  {
    ASSERT_EQ(gen1.initial_seed(), seed);
    MyRandomGenerator gen2(gen1);
    ASSERT_EQ(gen2.initial_seed(), seed);
    EXPECT_NE(gen1(), gen2());
  }
  {
    MyRandomGenerator gen2(gen3);
    EXPECT_EQ(gen2(), gen3());
    EXPECT_NE(gen1(), gen2());
    gen3();
    EXPECT_EQ(gen2(), gen3());
  }
}

TEST(discrete_random_variable, simple01) {
  const int itr = 300, seed = 10;
  ::gen::test::DiscreteRandomVariable<int> var(seed);
  var.Add(1, 0.33);
  var.Add(2, 0.47);
  var.Add(3, 0.2);
  int cnt[4] = {0};
  for (int i = 0; i < itr; ++i)
    cnt[var()]++;
  EXPECT_LE(cnt[1], cnt[2]);
  EXPECT_GE(cnt[1], cnt[3]);
  EXPECT_GE(cnt[2], cnt[3]);

  for (int i = 0; i < 3; ++i)
    std::cout << cnt[i + 1] << " ";
  std::cout << "\n";

  var.Reset(seed);
  for (int i = 0; i < itr; ++i)
    cnt[var()]--;
  EXPECT_EQ(0, cnt[1]);
  EXPECT_EQ(0, cnt[2]);
  EXPECT_EQ(0, cnt[3]);
}

TEST(discrete_range_uniform_random_variable, simple) {
  ::gen::test::DiscreteUniformRangeRandomVariable<int> var(0, 6, 3);
//  for (int i = 0; i < 300; ++i)
//    std::cout << var() << " ";
}

TEST(simple_weighted_graph_generator, uniform_edge_weight) {
  const unsigned seed = 2U;
  UniformEdgeWeightGenerator<> weight_gen(0, 100, seed);
  SimpleWeightedGraphGenerator<UniformEdgeWeightGenerator<> > swgg(
      5, weight_gen, seed);

  Edge<> e = NullEdge<unsigned>();
  bool adj_mat[5][5] = {{false}};
  for (int i = 0; i < 5 * 4; ++i) {
//    ASSERT_NO_THROW(e = swgg());
    ASSERT_FALSE(ksp::test::EdgeEqual<>()(e = swgg(), NullEdge<unsigned>()));
    std::cout << ::ksp::test::EToStr(e) << "\n";
    EXPECT_FALSE(adj_mat[e.tail][e.head]) << "edge " << e.tail << " -> " <<
        e.head << " already generated";
    adj_mat[e.tail][e.head] = true;
  }
//  EXPECT_THROW(swgg(), NoMoreElementsException);
  ASSERT_TRUE(ksp::test::EdgeEqual<>()(swgg(), NullEdge<unsigned>()));
}

TEST(simple_weighted_graph_generator, uniform_edge_weight_iter) {
  const unsigned seed = 2U;
  UniformEdgeWeightGenerator<> weight_gen(0, 100, seed);
  SimpleWeightedGraphGenerator<UniformEdgeWeightGenerator<> > swgg(
      5, weight_gen, seed), swgg2 = swgg;

  SimpleWeightedGraphGenerator<UniformEdgeWeightGenerator<> >::Iter it
  = swgg.begin();

  for (int i = 0; i < 5 * 4; ++i) {
    Edge<> e = swgg(), e2 = *(it);
//    ASSERT_TRUE(ksp::test::EdgeEqual<>()(e, e2));
    std::cout << ::ksp::test::EToStr(e) << " " << ::ksp::test::EToStr(e2) << "\n";
    ++it;
  }
}

TEST(weighted_multigraph_generator, uniform_edge_weight) {
  const unsigned seed = 2U;
  UniformEdgeWeightGenerator<> weight_gen(0, 100, seed);
  const unsigned n = 5;
  WeightedMultigraphGenerator<UniformEdgeWeightGenerator<> > wmgg(
      n, weight_gen, seed);

  Edge<> e = NullEdge<unsigned>();
  for (int itr = 0; itr < 100; ++itr) {
    ASSERT_NO_THROW(e = wmgg());
    ASSERT_LT(e.tail, n);
    ASSERT_LT(e.head, n);
//    std::cout << ::ksp::test::EToStr(e) << "\n";
  }
}

}  // namespace test

}  // namespace gen

}  // namespace ksp




