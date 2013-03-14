/*
 * Copyright (C) 2013 Leo Osvald <leo.osvald@gmail.com>
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

#ifndef RANDOM_SAMPLER_HPP_
#define RANDOM_SAMPLER_HPP_

#include <cstdlib>
#include <map>

#include "generator.hpp"
#include "random_utils.hpp"

namespace gen {

namespace test {

class RandomSampler : Generator<RandomSampler, std::size_t> {
  typedef Generator<RandomSampler, std::size_t> Base;
 public:
  RandomSampler(std::size_t n, unsigned seed);
  std::size_t operator()();

 private:
  typedef std::map<std::size_t, std::size_t> IndexMap;

  std::size_t cur_ind_;
  IndexMap ind_map_;
};

} // namespace test

} // namespace gen

#endif /* RANDOM_SAMPLER_HPP_ */
