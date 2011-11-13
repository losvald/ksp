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

#ifndef DISCRETE_RANDOM_VARIABLE_HPP_
#define DISCRETE_RANDOM_VARIABLE_HPP_

#include <exception>
#include <utility>
#include <vector>

#include "generator.hpp"
#include "random_utils.hpp"

namespace gen {

namespace test {

struct NoSuchOutcomeException : std::exception {
};

template<class T>
class DiscreteRandomVariable : public Generator<DiscreteRandomVariable<T>, T> {
  typedef Generator<DiscreteRandomVariable<T>, T> Base;
public:
  DiscreteRandomVariable(unsigned seed = NewSeedVal()) : Base(seed) { }

  T operator()() {
    if(sum_.empty())
      throw NoSuchOutcomeException();
    int total_sum = sum_.back();
    if(total_sum <= 0)
      throw NoSuchOutcomeException();
    int target = Random(std::make_pair(0, total_sum), Base::seed());
    int lo = 0, hi = sum_.size() - 1;
    while(lo < hi) {
      int mid = lo + (hi - lo - 1) / 2;
      if(target < sum_[mid])
        hi = mid;
      else
        lo = mid + 1;
    }
    return outcomes_[lo];
  }

  void Add(const T& outcome, double probability) {
    int s = AbsoluteProbability(probability) +
        (!sum_.empty() ? sum_.back() : 0);
    outcomes_.push_back(outcome);
    sum_.push_back(s);
  }

  void Remove(const T& outcome) {
    int ind = IndexOf(outcome);
    if(ind != -1) {
      int d = GetAbsoluteProbabilityAt(ind);
      sum_.erase(sum_.begin() + ind);
      outcomes_.erase(outcomes_.begin() + ind);
      Fix(ind, -d);
    }
  }

  void Clear() {
    outcomes_.clear();
    sum_.clear();
  }

  void SetProbability(T outcome, double probability) {
    short absolute_prob = AbsoluteProbability(probability);
    int ind = IndexOf(outcome);
    if(ind != -1) {
      Fix(ind, absolute_prob - GetAbsoluteProbabilityAt(ind));
    }
  }

  double GetProbability(const T& outcome) const {
    int ind = IndexOf(outcome);
    if(ind == -1)
      return 0;
    return static_cast<double>(GetAbsoluteProbabilityAt(ind) / sum_.back());
  }

  std::vector< std::pair<T, double> > GetDistribution() const {
    std::vector< std::pair<T, double> > d(outcomes_.size());
    for(std::size_t i = 0; i < outcomes_.size(); ++i) {
      d[i] = std::make_pair(outcomes_[i], GetProbability(outcomes_[i]));
    }
    return d;
  }

private:
  short AbsoluteProbability(double probability) const {
    if(probability < 0) return 0;
    if(probability >= 1) return kMaxAbsoluteProbability;
    return static_cast<short>(probability * kMaxAbsoluteProbability);
  }

  double RelativeProbability(short absolute_probability) const {
    return static_cast<double>(absolute_probability) / kMaxAbsoluteProbability;
  }

  void Fix(int from, int sum_diff) {
    for(std::size_t i = from; i < sum_.size(); ++i)
      sum_[i] += sum_diff;
  }

  int GetAbsoluteProbabilityAt(int index) const {
    return sum_[index] - (index > 0 ? sum_[index - 1] : 0);
  }

  int IndexOf(const T& outcome) const {
    for(std::size_t i = 0; i < outcomes_.size(); ++i)
      if(outcomes_[i] == outcome)
        return i;
    return -1;
  }

  std::vector<int> sum_;
  std::vector<T> outcomes_;
  static const short kMaxAbsoluteProbability = 0x7fff;
};


template<typename T>
class DiscreteUniformRangeRandomVariable :
  Generator<DiscreteUniformRangeRandomVariable<T>, T> {
  typedef Generator<DiscreteUniformRangeRandomVariable<T>, T> Base;
public:
  DiscreteUniformRangeRandomVariable(const T& from, const T& to,
                                     unsigned seed = NewSeedVal())
  : Base(seed), from(from), to(to) { }

  inline T operator()() {
    return Random(from, to, Base::seed());
  }

  T from;
  T to;
};

}  // namespace test

}  // namespace gen

#endif /* DISCRETE_RANDOM_VARIABLE_HPP_ */
