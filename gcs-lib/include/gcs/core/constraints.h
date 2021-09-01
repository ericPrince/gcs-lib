#ifndef GCS_LIB_INCLUDE_GCS_CORE_CONSTRAINTS
#define GCS_LIB_INCLUDE_GCS_CORE_CONSTRAINTS

#include <ceres/ceres.h>
#include <gcs/core/solve_elements.h>

#include <boost/preprocessor.hpp>
#include <cmath>
#include <metal.hpp>

namespace gcs {

struct Constraint {
    // virtual std::vector<Equation> get_equations() = 0;  // TODO

    virtual void add_cost_function(ceres::Problem& problem) = 0;
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

struct set_const_functor {
    double const_value;

    static const metal::int_ num_params = 1;

    template <typename T>
    bool operator()(const T* x, T* residual) const {
        *residual = T{*x} - const_value;
        return true;
    }
};

struct SetConstant : Constraint {
    double value;
    Variable* v;
    Equation eqn;

    SetConstant(Variable& v, double value) : v{&v}, value{value}, eqn{{&v}} {}

    void add_cost_function(ceres::Problem& problem);
};

//! @brief Set two values to equal
template <typename T>
T equate(const T& x1, const T& x2) {
    return x1 - x2;
}

CSTR_CREATE_FUNCTOR_(equate, 2);

struct Equate : Constraint {
    Variable* v1;
    Variable* v2;
    Equation eqn;

    Equate(Variable& v1, Variable& v2) : v1{&v1}, v2{&v2}, eqn{{&v1, &v2}} {}

    void add_cost_function(ceres::Problem& problem);
};

//! @brief Set the difference between 2 values
template <typename T>
T difference(const T& x1, const T& x2, const T& d) {
    return ceres::abs(x1 - x2) - d;
}

CSTR_CREATE_FUNCTOR_(difference, 3);

struct Difference : Constraint {
    Variable* v1;
    Variable* v2;
    Variable* d;
    Equation eqn;

    Difference(Variable& v1, Variable& v2, Variable& d)
        : v1{&v1}, v2{&v2}, d{&d}, eqn{{&v1, &v2, &d}} {}

    void add_cost_function(ceres::Problem& problem);
};

}  // namespace gcs

#endif  // GCS_LIB_INCLUDE_GCS_CORE_CONSTRAINTS
