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

#include <cstddef>
#include <ctime>

#include "random_utils.hpp"

namespace gen {

namespace test {

unsigned kDefaultSeedVal = NewSeedVal();
unsigned* gSeed = &kDefaultSeedVal;

unsigned NewSeedVal() {
  return NewSeedVal(&kDefaultSeedVal);
}

unsigned NewSeedVal(const unsigned* address_offset) {
  return (unsigned)((unsigned)time(NULL) + (std::size_t)address_offset);
}

}  // namespace test

}  // namespace gen

