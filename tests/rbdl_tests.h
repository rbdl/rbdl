//
// Created by martin on 13.04.20.
//

#ifndef RBDL_RBDL_TESTS_H
#define RBDL_RBDL_TESTS_H

#include "catch.hpp"

template<typename T>
struct ApproxVectorMatcher : Catch::MatcherBase<T> {

    ApproxVectorMatcher(T const& comparator) : m_comparator(comparator ) {}

    bool match(T const &v) const override {
      if (m_comparator.size() != v.size())
        return false;
      for (std::size_t i = 0; i < v.size(); ++i)
        if (m_comparator[i] != approx(v[i]))
          return false;
      return true;
    }
    std::string describe() const override {
      return "is approx: " + ::Catch::Detail::stringify( m_comparator );
    }
    template <typename = typename std::enable_if<std::is_constructible<double, typename T::Scalar>::value>::type>
    ApproxVectorMatcher& epsilon(double const& newEpsilon ) {
      approx.epsilon(newEpsilon);
      return *this;
    }
    template <typename = typename std::enable_if<std::is_constructible<double, typename T::Scalar>::value>::type>
    ApproxVectorMatcher& margin(double const& newMargin ) {
      approx.margin(newMargin);
      return *this;
    }
    template <typename = typename std::enable_if<std::is_constructible<double, typename T::Scalar>::value>::type>
    ApproxVectorMatcher& scale(double const& newScale ) {
      approx.scale(newScale);
      return *this;
    }

    T const& m_comparator;
    mutable Catch::Detail::Approx approx = Catch::Detail::Approx::custom();
};

template<typename T>
ApproxVectorMatcher<T> IsApprox( T const& comparator ) {
  return ApproxVectorMatcher<T>( comparator );
}

#endif //RBDL_RBDL_TESTS_H
