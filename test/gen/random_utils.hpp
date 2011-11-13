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

#ifndef RANDOM_UTILS_HPP_
#define RANDOM_UTILS_HPP_

#include <cstdlib>

#include <exception>
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace gen {

namespace test {

unsigned NewSeedVal();
unsigned NewSeedVal(const unsigned* address_offset);

extern unsigned kDefaultSeedVal;
extern unsigned* gSeed;

namespace {

template<int RandMax>
static inline int GetRandMaxDigits0() {
#if (RAND_MAX + 1) == ((RAND_MAX + 1) & -(RAND_MAX + 1))
  int ret = 1;
#else
  int ret = 0;
#endif
  for (int x = RAND_MAX; x >>= 1; )
    ++ret;
  return ret;
}

template<>
inline int GetRandMaxDigits0<32767>() {
  return 8;
}

template<>
inline int GetRandMaxDigits0<(1U << std::numeric_limits<int>::digits) - 1>() {
  return std::numeric_limits<int>::digits;
}

const unsigned kRandMaxMask = RAND_MAX + 1U;
const int kRandMaxDigits = GetRandMaxDigits0<RAND_MAX>();


}  // namespace

class BadRandomRangeException {
};

template<typename T>
static T Random(unsigned* seed) {
  const int t_bits = std::numeric_limits<T>::digits;
  const int r_bits = t_bits < kRandMaxDigits ? t_bits : kRandMaxDigits;
  const int r_mask = (1U << r_bits) - 1;

  T r = T();
  int bits;
  for (bits = t_bits; bits >= r_bits; bits -= r_bits)
    r = (r << r_bits) | (rand_r(seed) & r_mask);
  if (bits)
    r = (r << bits) | (rand_r(seed) & ((1U << bits) - 1));
  return r;
}

template<typename T>
static inline T Random(const T& to, unsigned* seed = gSeed) {
  return Random<T>(seed) % to;
}

template<typename T>
static T Random(const T& from, const T& to, unsigned* seed = gSeed) {
  RangeCheck(from, to);
  return Random<T>(seed) % (to - from) + from;
}

template<typename T>
static inline T Random(const std::pair<T, T>& range, unsigned* seed = gSeed) {
  return Random<T>(range.first, range.second, seed);
}

template<typename T>
static std::vector<T> RandomVector(std::size_t n, const T& from, const T& to,
                                   unsigned* seed = gSeed) {
  std::vector<T> v(n);
  for(std::size_t i = 0; i < n; ++i)
    v[i] = Random<T>(from, to, seed);
  return v;
}

template<typename T>
static inline std::vector<T> RandomVector(std::size_t n,
                                          const std::pair<T, T>& range,
                                          unsigned* seed = gSeed) {
  return RandomVector<T>(n, range.first, range.second, seed);
}


namespace {

template<typename T>
inline void RangeCheck(const T& from, const T& to) {
  if(to <= from)
    throw BadRandomRangeException();
}

}  // namespace

}  // namespace test

}  // namespace gen

#endif /* RANDOM_UTILS_HPP_ */
