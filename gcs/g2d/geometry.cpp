#include "gcs/g2d/geometry.h"

#include <vector>

namespace gcs {

namespace g2d {

std::vector<Variable*> Point::get_variables() { return {&this->x, &this->y}; }

std::vector<Variable*> Line::get_variables() {
    return {&this->p1.x, &this->p1.y, &this->p2.x, &this->p2.y};
}

std::vector<Variable*> Circle::get_variables() {
    return {&this->p.x, &this->p.y, &this->r};
}

}  // namespace g2d

}  // namespace gcs