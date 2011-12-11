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

#ifndef TEST_HPP_
#define TEST_HPP_

//#define SKIP_TESTS 1
#define VERBOSITY 1

#include <algorithm>

#include <gtest/gtest.h>

extern bool gDebugTT;

template<class InputIterator1, class InputIterator2, class Equals>
bool CheckEq(InputIterator1 exp_begin, InputIterator1 exp_end,
             InputIterator2 act_begin, InputIterator2 act_end,
             const Equals& equals) {
  InputIterator1 exp_it = exp_begin;
  InputIterator2 act_it = act_begin;
  for (; exp_it != exp_end && act_it != act_end ; ++exp_it, ++act_it) {
    if (!equals(*exp_it, *act_it))
      return false;
  }
  return exp_it == exp_end && act_it == act_end;
}

template<class InputIterator1, class InputIterator2, class Compare,
class Equals>
bool CheckSortedEq(InputIterator1 exp_begin, InputIterator1 exp_end,
              InputIterator2 act_begin, InputIterator2 act_end,
              const Compare& comp,
              const Equals& equals) {
  std::sort(exp_begin, exp_end, comp);
  std::sort(act_begin, act_end, comp);
  return CheckEq(exp_begin, exp_end, act_begin, act_end, equals);
}

#endif /* TEST_HPP_ */
