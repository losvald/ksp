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

#include "random_sampler.hpp"

namespace gen {

namespace test {

RandomSampler::RandomSampler(std::size_t n, unsigned seed = NewSeedVal())
    : Base(seed, n),
      cur_ind_(n) {
}

std::size_t RandomSampler::operator()() {
  if (cur_ind_ == 0)
    return -1;

  // swap cur_ind_ - 1 with a random index ind from range [0, cur_ind)
  std::size_t ind = ::gen::test::Random(cur_ind_--, Base::seed());
  IndexMap::iterator i = ind_map_.find(ind);
  std::size_t& ind_val = (i != ind_map_.end() ? i->second
                          : ind_map_[ind] = ind);
  std::size_t ind_val_before_swap = ind_val;
  if ((i = ind_map_.find(cur_ind_)) != ind_map_.end()) {
    ind_val = i->second;
    ind_map_.erase(i);
  } else
    ind_val = cur_ind_;
  return ind_val_before_swap;
}

} // namespace test

} // namespace gen
