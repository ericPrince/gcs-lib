#include <vector>

#include <gcs/geometry/geometry.h>


namespace GCS {


std::vector<Variable*> Point::get_variables() {
    return { &this->x, &this->y };
}


std::vector<Variable*> Line::get_variables() {
    return { &this->p1.x, &this->p1.y, &this->p2.x, &this->p2.y };
}


std::vector<Variable*> Circle::get_variables() {
    return { &this->p.x, &this->p.y, &this->r };
}


//-------------------------------------------------------------------


//std::vector<Equation> SetConstant::get_equations() {
//    return { Equation{std::unordered_set<Variable*>{ this->v }} };
//}
//
//
//std::vector<Equation> Distance::get_equations() {
//    return { Equation{std::unordered_set<Variable*>{ &this->p1->x, &this->p1->y }} };
//}
//
//
//std::vector<Equation> Equate::get_equations() {
//    return { Equation{std::unordered_set<Variable*>{ this->v1, this->v2 }} };
//}


}  // namespace GCS
