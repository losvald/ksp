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

#include "edge.hpp"

#include "util/string_utils.hpp"

namespace ksp {

namespace {

std::string EdgeIdToStr(const EdgeId* e) {
  if (e == NULL)
    return "(unknown)";
  return MakeString() << *e;
}

}  //  namespace


NoSuchEdgeException::NoSuchEdgeException() { }

NoSuchEdgeException::NoSuchEdgeException(EdgeId e) : EdgeException(e) {
}

const char* NoSuchEdgeException::what() const throw() {
  return MakeString() << "Edge " << EdgeIdToStr(e_) << " does not exist";
}

}  // namespace ksp
