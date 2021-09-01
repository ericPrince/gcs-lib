#include <gcs/g2d/constraints_unsigned.h>
#include <gcs/g2d/constraints_unsigned_math.h>

namespace gcs {

namespace g2d {

void Distance::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(create_scalar_autodiff(new distance_functor{}),
                             nullptr,
                             &this->p1->x.value,
                             &this->p1->y.value,
                             &this->p2->x.value,
                             &this->p2->y.value,
                             &this->d->value);
}

void PointOnLine::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new point_on_line_functor{}),
        nullptr,
        &this->L->p1.x.value,
        &this->L->p1.y.value,
        &this->L->p2.x.value,
        &this->L->p2.y.value,
        &this->p->x.value,
        &this->p->y.value);
}

void OffsetLinePoint::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new offset_line_point_functor{}),
        nullptr,
        &this->L->p1.x.value,
        &this->L->p1.y.value,
        &this->L->p2.x.value,
        &this->L->p2.y.value,
        &this->p->x.value,
        &this->p->y.value,
        &this->d->value);
}

void AngleBetweenLines::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new angle_point_4_functor{}),
        nullptr,
        &this->L1->p1.x.value,
        &this->L1->p1.y.value,
        &this->L1->p2.x.value,
        &this->L1->p2.y.value,
        &this->L2->p1.x.value,
        &this->L2->p1.y.value,
        &this->L2->p2.x.value,
        &this->L2->p2.y.value,
        &this->a->value);
}

void AngleThreePoints::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new angle_point_3_functor{}),
        nullptr,
        &this->p1->x.value,
        &this->p1->y.value,
        &this->p2->x.value,
        &this->p2->y.value,
        &this->p3->x.value,
        &this->p3->y.value,
        &this->a->value);
}

void AngleLine::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new angle_point_2_functor{}),
        nullptr,
        &this->L->p1.x.value,
        &this->L->p1.y.value,
        &this->L->p2.x.value,
        &this->L->p2.y.value,
        &this->a->value);
}

void PointOnCircle::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new point_on_circle_functor{}),
        nullptr,
        &this->p->x.value,
        &this->p->y.value,
        &this->c->p.x.value,
        &this->c->p.y.value,
        &this->c->r.value);
}

void TangentLineCircle::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new tangent_line_circle_functor{}),
        nullptr,
        &this->L->p1.x.value,
        &this->L->p1.y.value,
        &this->L->p2.x.value,
        &this->L->p2.y.value,
        &this->c->p.x.value,
        &this->c->p.y.value,
        &this->c->r.value);
}

void LineLength::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(create_scalar_autodiff(new line_length_functor{}),
                             nullptr,
                             &this->L->p1.x.value,
                             &this->L->p1.y.value,
                             &this->L->p2.x.value,
                             &this->L->p2.y.value,
                             &this->d->value);
}

void TangentCircles::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        create_scalar_autodiff(new tangent_circles_functor{}),
        nullptr,
        &this->c1->p.x.value,
        &this->c1->p.y.value,
        &this->c1->r.value,
        &this->c2->p.x.value,
        &this->c2->p.y.value,
        &this->c2->r.value);
}

}  // namespace g2d

}  // namespace gcs
