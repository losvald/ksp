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

#ifndef GEN_ITER_HPP_
#define GEN_ITER_HPP_

#include <cstddef>

#include <iterator>

namespace gen {

namespace test {

template<class G, typename T>
class GeneratorIterator : public std::iterator<std::input_iterator_tag,
ptrdiff_t> {
public:
  GeneratorIterator(const G& gen = G()) : gen_(gen), n_(0), last_(gen_()) { }

  inline const T& operator*() const {
    return last_;
  }

  inline const T* operator->() const {
    return &last_;
  }

  inline GeneratorIterator& operator++() {
    Increment();
    return *this;
  }

  inline GeneratorIterator operator++(int) {
    GeneratorIterator old = *this;
    Increment();
    return *this;
  }

  inline friend bool operator==(const GeneratorIterator& lhs,
      const GeneratorIterator& rhs) {
    return lhs.n_ == rhs.n_;
  }

  inline friend bool operator!=(const GeneratorIterator& lhs,
      const GeneratorIterator& rhs) {
    return !(lhs == rhs);
  }

  inline friend ptrdiff_t operator-(const GeneratorIterator& lhs,
                                    const GeneratorIterator& rhs) {
    return lhs.n_ - rhs.n_;
  }

  GeneratorIterator& Skip(unsigned n_pos) {
    n_ += n_pos;
    return *this;
  }

  GeneratorIterator& Rewind(unsigned n_pos) {
    n_ = (n_ >= n_pos ? n_ - n_pos : 0);
    return *this;
  }

private:
  inline void Increment() {
    last_ = gen_();
    ++n_;
  }

  G gen_;
  std::size_t n_;
  T last_;
};


template<class G, typename T>
class GeneratorIteratorProvider {
public:
  typedef GeneratorIterator<G, T> Iter;

  GeneratorIteratorProvider(unsigned max_itr = -1) : max_itr_(max_itr) { }

  Iter begin() const {
    return Iter(*static_cast<const G*>(this));
  }

  Iter end() const {
    return Iter(*static_cast<const G*>(this)).Skip(max_itr_ - 1U);
  }

protected:
  typedef GeneratorIteratorProvider ProviderType;

  unsigned max_itr() const {
    return max_itr_;
  }

private:
  unsigned max_itr_;
};


}  // namespace test

}  // namespace gen


namespace std {

template<class G, typename T>
typename ::gen::test::GeneratorIterator<G, T>::difference_type
distance(::gen::test::GeneratorIterator<G, T> first,
         ::gen::test::GeneratorIterator<G, T> last) {
  return last - first;
}

}  // namespace std

#endif /* GEN_ITER_HPP_ */
