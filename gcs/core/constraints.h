#ifndef GCS_CORE_CONSTRAINTS
#define GCS_CORE_CONSTRAINTS

#include <ceres/ceres.h>

#include <boost/preprocessor.hpp>
#include <cmath>
#include <metal.hpp>

#include "gcs/core/solve_elements.h"

namespace gcs {

struct Constraint {
    // virtual std::vector<Equation> get_equations() = 0;  // TODO

    virtual void add_to_problem(ceres::Problem& problem) = 0;

    virtual std::vector<gcs::Equation*> get_equations() const = 0;
};

#ifndef CSTR_CREATE_FUNCTOR_
#define CSTR_CREATE_FUNCTOR_(func, n)                          \
    struct func##_functor {                                    \
        static const metal::int_ num_params = n;               \
                                                               \
        template <typename T>                                  \
        bool operator()(BOOST_PP_ENUM_PARAMS(n, const T* x)    \
                            BOOST_PP_COMMA_IF(n) T* r) const { \
            *r = func(BOOST_PP_ENUM_PARAMS(n, *x));            \
            return true;                                       \
        }                                                      \
    };
#endif

namespace detail {

template <typename Functor, typename... Args>
ceres::CostFunction* create_ad_impl_(Functor* functor, metal::list<Args...>) {
    return new ceres::AutoDiffCostFunction<Functor, Args::value...>(functor);
}

}  // namespace detail

template <typename Functor>
ceres::CostFunction* create_scalar_autodiff(Functor* functor) {
    using param_list =
        metal::repeat<metal::number<1>, metal::number<Functor::num_params + 1>>;

    return detail::create_ad_impl_(functor, param_list{});
}

}  // namespace gcs

#endif  // GCS_CORE_CONSTRAINTS
