#include <ceres/ceres.h>
#include "gcs/core/constraints.h"

namespace gcs {

void SetConstant::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new set_const_functor{this->value}),
        nullptr,
        &this->v->value);
}

void Equate::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(create_scalar_autodiff(new equate_functor{}),
                             nullptr,
                             &this->v1->value,
                             &this->v2->value);
}

void Difference::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(create_scalar_autodiff(new difference_functor{}),
                             nullptr,
                             &this->v1->value,
                             &this->v2->value,
                             &this->d->value);
}

}  // namespace gcs
