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

#ifndef EDGE_GENERATOR_HPP_
#define EDGE_GENERATOR_HPP_

#include "discrete_random_variable.hpp"

namespace ksp {

namespace gen {

namespace test {

template<typename T = unsigned>
class UniformEdgeWeightGenerator
: public ::gen::test::DiscreteUniformRangeRandomVariable<T> {
  typedef ::gen::test::DiscreteUniformRangeRandomVariable<T> Base;
public:
  UniformEdgeWeightGenerator(const T& min_weight, const T& max_weight,
                             unsigned seed = ::gen::test::NewSeedVal())
  : Base(min_weight, max_weight + 1, seed) { }
};

}  // namespace test

}  // namespace gen

}  // namespace ksp


#endif /* EDGE_GENERATOR_HPP_ */
