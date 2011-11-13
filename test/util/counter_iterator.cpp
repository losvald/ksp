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
#include <numeric>

#include "../test.hpp"

#include "../../src/util/counter_iterator.hpp"

#ifndef SKIP_TESTS

namespace test {

typedef CounterIterator<int> CIter;

TEST(counter_iterator, simple01) {
  int from = -5, to = 7, x = from;
  CIter fromi(from), toi(to);
  for (CIter i = fromi; i != toi; ++i, ++x)
    EXPECT_EQ(x, *i);
  EXPECT_EQ(-6, *(--fromi));
}

TEST(counter_iterator, accumulate) {
  EXPECT_EQ(18, std::accumulate(CIter(5), CIter(8), 0));
  for (int n = 0; n < 100; ++n)
    ASSERT_EQ(n * (n - 1) / 2, std::accumulate(CIter(0), CIter(n), 0));
}


}  // namespace test


#endif

