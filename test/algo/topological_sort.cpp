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

#include "../../src/algo/topological_sort.hpp"
#include "../../src/graph_builder.hpp"
#include "../../src/util/string_utils.hpp"
#include "../../src/vertex_map.hpp"

#include "../test.hpp"

#include "../graph_examples.hpp"
#include "../shortest_path_tree_examples.hpp"

namespace ksp {

namespace test {

namespace {

const GraphBuilder<char> kGBLeda = example::graph::LedaShortestPaths();
const ShortestPathTree<> kSptLeda = example::spt::LedaShortestPaths();

template<typename T, typename VertexInputIterator, typename EdgeInputIterator>
bool CheckTopological(VertexInputIterator v_begin, VertexInputIterator v_end,
                      EdgeInputIterator e_begin, EdgeInputIterator e_end) {
  VertexMap<std::size_t> ord(*std::max_element(v_begin, v_end) + 1);
  for (std::size_t ind = 0; v_begin != v_end; ++v_begin) {
    ord[*v_begin] = ind++;
  }
  for (; e_begin != e_end; ++e_begin)
    if (ord[e_begin->tail] > ord[e_begin->head])
      return false;
  return true;
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

}  // namespace

TEST(topological_sort, Leda) {
  std::vector<VertexId> v_sorted;
  EXPECT_TRUE(TopologicalSort<unsigned>(
      kSptLeda.edges_begin(),kSptLeda.edges_end(), &v_sorted));

//  std::cout << TopoToStr(v_sorted.begin(), v_sorted.end(), kGBLeda) << std::endl;

  ASSERT_TRUE(CheckTopological<unsigned>(
      v_sorted.begin(), v_sorted.end(),
      kSptLeda.edges_begin(), kSptLeda.edges_end())) <<
      "Actual: " << TopoToStr(v_sorted.begin(), v_sorted.end(), kGBLeda) <<
      std::endl;
}


}  // namespace test

}  // namespace ksp
