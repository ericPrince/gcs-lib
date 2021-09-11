#ifndef GCS_BASIC_CONSTRAINTS
#define GCS_BASIC_CONSTRAINTS

#include <ceres/ceres.h>

#include <metal.hpp>
#include <vector>

#include "gcs/basic/constraint_math.h"
#include "gcs/core/core.h"

namespace gcs {

namespace basic {

struct SetConstant : gcs::Constraint {
    gcs::Variable* var;
    double value;

    SetConstant(gcs::Variable& var, double value) : var{&var}, value{value} {}

    struct Functor_0 {
        static const metal::int_ num_params = 1;
        double value;

        template <typename T>
        bool operator()(const T* var, T* r) const {
            *r = equate(*var, value);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(
            gcs::create_scalar_autodiff(new Functor_0{value}),
            nullptr,
            &var->value);
    }

    std::vector<gcs::Equation*> get_equations() const {
        std::vector<gcs::Equation*> eqns{};

        eqns.push_back(new Equation{
            {var}, [&](ceres::Problem& problem) {
                problem.AddResidualBlock(
                    gcs::create_scalar_autodiff(new Functor_0{value}),
                    nullptr,
                    &var->value);
            }});

        return eqns;
    }
};

struct Equate : gcs::Constraint {
    gcs::Variable* v1;
    gcs::Variable* v2;

    Equate(gcs::Variable& v1, gcs::Variable& v2) : v1{&v1}, v2{&v2} {}

    struct Functor_0 {
        static const metal::int_ num_params = 2;

        template <typename T>
        bool operator()(const T* v1, const T* v2, T* r) const {
            *r = equate(*v1, *v2);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &v1->value,
                                 &v2->value);
    }

    std::vector<gcs::Equation*> get_equations() const {
        std::vector<gcs::Equation*> eqns{};

        eqns.push_back(
            new Equation{{v1, v2}, [&](ceres::Problem& problem) {
                             problem.AddResidualBlock(
                                 gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &v1->value,
                                 &v2->value);
                         }});

        return eqns;
    }
};

struct Difference : gcs::Constraint {
    gcs::Variable* v1;
    gcs::Variable* v2;
    gcs::Variable* diff;

    Difference(gcs::Variable& v1, gcs::Variable& v2, gcs::Variable& diff)
        : v1{&v1}, v2{&v2}, diff{&diff} {}

    struct Functor_0 {
        static const metal::int_ num_params = 3;

        template <typename T>
        bool operator()(const T* v1, const T* v2, const T* diff, T* r) const {
            *r = difference(*v1, *v2, *diff);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &v1->value,
                                 &v2->value,
                                 &diff->value);
    }

    std::vector<gcs::Equation*> get_equations() const {
        std::vector<gcs::Equation*> eqns{};

        eqns.push_back(
            new Equation{{v1, v2, diff}, [&](ceres::Problem& problem) {
                             problem.AddResidualBlock(
                                 gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &v1->value,
                                 &v2->value,
                                 &diff->value);
                         }});

        return eqns;
    }
};

}  // namespace basic

}  // namespace gcs

#endif  // GCS_BASIC_CONSTRAINTS