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

#ifndef GENERATOR_HPP_
#define GENERATOR_HPP_

#include "gen_iter.hpp"

namespace gen {

namespace test {

template<typename G, typename T>
class Generator : public GeneratorIteratorProvider<G, T> {
  typedef GeneratorIteratorProvider<G, T> Base;
public:
  inline void Reset(unsigned initial_seed) {
    seed_ = initial_seed_ = initial_seed;
  }

  inline unsigned initial_seed() const {
    return initial_seed_;
  }

protected:
  typedef Generator GeneratorType;

  Generator(unsigned seed, unsigned max_itr = -1) : Base(max_itr) {
    Reset(seed);
  }

  Generator(const Generator& gen) : Base(gen.max_itr()) {
    Reset(gen.initial_seed_);
  }

  inline unsigned* seed() {
    return &seed_;
  }

private:
  Generator& operator=(const Generator&);

  unsigned initial_seed_;
  unsigned seed_;
};

}  // namespace test

}  // namespace gen

#endif /* GENERATOR_HPP_ */
