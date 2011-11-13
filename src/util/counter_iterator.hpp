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

#ifndef COUNTER_ITERATOR_HPP_
#define COUNTER_ITERATOR_HPP_

#include <iterator>

template<typename T>
class CounterIterator : public std::iterator<std::input_iterator_tag,
ptrdiff_t> {
public:
  inline CounterIterator(const T& incrementable = T())
  : counter_(incrementable) { }

  inline const T& operator*() const {
    return counter_;
  }

  inline const T* operator->() const {
    return &counter_;
  }

  inline CounterIterator& operator++() {
    ++counter_;
    return *this;
  }

  inline CounterIterator& operator--() {
    --counter_;
    return *this;
  }

  inline friend bool operator==(const CounterIterator& lhs,
      const CounterIterator& rhs) {
    return lhs.counter_ == rhs.counter_;
  }

  inline friend bool operator!=(const CounterIterator& lhs,
      const CounterIterator& rhs) {
    return !(lhs == rhs);
  }

private:
  T counter_;

};

#endif /* COUNTER_ITERATOR_HPP_ */
