#include <gcs/constraints/constraints2d_unsigned.h>
#include <gcs/geometry/geometry.h>

#include <vector>

namespace GCS {

namespace cstr = constraints2d_unsigned;

std::vector<Variable*> Point::get_variables() { return {&this->x, &this->y}; }

std::vector<Variable*> Line::get_variables() {
    return {&this->p1.x, &this->p1.y, &this->p2.x, &this->p2.y};
}

std::vector<Variable*> Circle::get_variables() {
    return {&this->p.x, &this->p.y, &this->r};
}

//-------------------------------------------------------------------

void SetConstant::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::set_const_functor{this->value}),
        nullptr,
        &this->v->value);
}

void Distance::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::distance_functor{}),
        nullptr,
        &this->p1->x.value,
        &this->p1->y.value,
        &this->p2->x.value,
        &this->p2->y.value,
        &this->d->value);
}

void Equate::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::equate_functor{}),
        nullptr,
        &this->v1->value,
        &this->v2->value);
}

void Difference::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::difference_functor{}),
        nullptr,
        &this->v1->value,
        &this->v2->value,
        &this->d->value);
}

void PointOnLine::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::point_on_line_functor{}),
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
        cstr::create_scalar_autodiff(new cstr::offset_line_point_functor{}),
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
        cstr::create_scalar_autodiff(new cstr::angle_point_4_functor{}),
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
        cstr::create_scalar_autodiff(new cstr::angle_point_3_functor{}),
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
        cstr::create_scalar_autodiff(new cstr::angle_point_2_functor{}),
        nullptr,
        &this->L->p1.x.value,
        &this->L->p1.y.value,
        &this->L->p2.x.value,
        &this->L->p2.y.value,
        &this->a->value);
}

void PointOnCircle::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::point_on_circle_functor{}),
        nullptr,
        &this->p->x.value,
        &this->p->y.value,
        &this->c->p.x.value,
        &this->c->p.y.value,
        &this->c->r.value);
}

void TangentLineCircle::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::tangent_line_circle_functor{}),
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
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::line_length_functor{}),
        nullptr,
        &this->L->p1.x.value,
        &this->L->p1.y.value,
        &this->L->p2.x.value,
        &this->L->p2.y.value,
        &this->d->value);
}

void TangentCircles::add_cost_function(ceres::Problem& problem) {
    problem.AddResidualBlock(
        cstr::create_scalar_autodiff(new cstr::tangent_circles_functor{}),
        nullptr,
        &this->c1->p.x.value,
        &this->c1->p.y.value,
        &this->c1->r.value,
        &this->c2->p.x.value,
        &this->c2->p.y.value,
        &this->c2->r.value);
}

// std::vector<Equation> SetConstant::get_equations() {
//    return { Equation{std::unordered_set<Variable*>{ this->v }} };
//}
//
//
// std::vector<Equation> Distance::get_equations() {
//    return { Equation{std::unordered_set<Variable*>{ &this->p1->x,
//    &this->p1->y }} };
//}
//
//
// std::vector<Equation> Equate::get_equations() {
//    return { Equation{std::unordered_set<Variable*>{ this->v1, this->v2 }}
//    };
//}

}  // namespace GCS
