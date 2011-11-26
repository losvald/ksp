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

#ifndef BIT_UTILS_HPP_
#define BIT_UTILS_HPP_

template<typename T>
static unsigned BitCount(const T& x) {
  unsigned ret = 0;
  for (; x; x >>= 1)
    ++ret;
  return ret;
}

template<typename T>
static unsigned SetBitCount(T x) {
  unsigned ret = 0;
  for (; x; x &= x - 1)
    ++ret;
  return ret;
}

#endif /* BIT_UTILS_HPP_ */
