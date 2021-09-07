#ifndef GCS_G2D_CONSTRAINTS
#define GCS_G2D_CONSTRAINTS

#include <ceres/ceres.h>

#include <metal.hpp>
#include <vector>

#include "gcs/core/core.h"
#include "gcs/g2d/constraints_unsigned_math.h"
#include "gcs/g2d/geometry.h"

namespace gcs {

namespace g2d {

struct PointOnLine : gcs::Constraint {
    gcs::g2d::Point* point;
    gcs::g2d::Line* line;

    PointOnLine(gcs::g2d::Point& point, gcs::g2d::Line& line)
        : point{&point}, line{&line} {}

    struct Functor_0 {
        static const metal::int_ num_params = 6;

        template <typename T>
        bool operator()(const T* point_x,
                        const T* point_y,
                        const T* line_p1_x,
                        const T* line_p1_y,
                        const T* line_p2_x,
                        const T* line_p2_y,
                        T* r) const {
            *r = point_on_line(*point_x,
                               *point_y,
                               *line_p1_x,
                               *line_p1_y,
                               *line_p2_x,
                               *line_p2_y);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &point->x.value,
                                 &point->y.value,
                                 &line->p1.x.value,
                                 &line->p1.y.value,
                                 &line->p2.x.value,
                                 &line->p2.y.value);
    }
};

struct CoincidentPoints : gcs::Constraint {
    gcs::g2d::Point* p1;
    gcs::g2d::Point* p2;

    CoincidentPoints(gcs::g2d::Point& p1, gcs::g2d::Point& p2)
        : p1{&p1}, p2{&p2} {}

    struct Functor_0 {
        static const metal::int_ num_params = 2;

        template <typename T>
        bool operator()(const T* p1_x, const T* p2_x, T* r) const {
            *r = equate(*p1_x, *p2_x);
            return true;
        }
    };
    struct Functor_1 {
        static const metal::int_ num_params = 2;

        template <typename T>
        bool operator()(const T* p1_y, const T* p2_y, T* r) const {
            *r = equate(*p1_y, *p2_y);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &p1->x.value,
                                 &p2->x.value);
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_1{}),
                                 nullptr,
                                 &p1->y.value,
                                 &p2->y.value);
    }
};

struct DistancePoints : gcs::Constraint {
    gcs::g2d::Point* p1;
    gcs::g2d::Point* p2;
    gcs::Variable* d;

    DistancePoints(gcs::g2d::Point& p1, gcs::g2d::Point& p2, gcs::Variable& d)
        : p1{&p1}, p2{&p2}, d{&d} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template <typename T>
        bool operator()(const T* p1_x,
                        const T* p1_y,
                        const T* p2_x,
                        const T* p2_y,
                        const T* d,
                        T* r) const {
            *r = distance(*p1_x, *p1_y, *p2_x, *p2_y, *d);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &p1->x.value,
                                 &p1->y.value,
                                 &p2->x.value,
                                 &p2->y.value,
                                 &d->value);
    }
};

struct LineLength : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::Variable* d;

    LineLength(gcs::g2d::Line& line, gcs::Variable& d) : line{&line}, d{&d} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template <typename T>
        bool operator()(const T* line_p1_x,
                        const T* line_p1_y,
                        const T* line_p2_x,
                        const T* line_p2_y,
                        const T* d,
                        T* r) const {
            *r = distance(*line_p1_x, *line_p1_y, *line_p2_x, *line_p2_y, *d);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x.value,
                                 &line->p1.y.value,
                                 &line->p2.x.value,
                                 &line->p2.y.value,
                                 &d->value);
    }
};

struct OffsetLinePoint : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::g2d::Point* point;
    gcs::Variable* d;

    OffsetLinePoint(gcs::g2d::Line& line,
                    gcs::g2d::Point& point,
                    gcs::Variable& d)
        : line{&line}, point{&point}, d{&d} {}

    struct Functor_0 {
        static const metal::int_ num_params = 7;

        template <typename T>
        bool operator()(const T* line_p1_x,
                        const T* line_p1_y,
                        const T* line_p2_x,
                        const T* line_p2_y,
                        const T* point_x,
                        const T* point_y,
                        const T* d,
                        T* r) const {
            *r = offset_line_point(*line_p1_x,
                                   *line_p1_y,
                                   *line_p2_x,
                                   *line_p2_y,
                                   *point_x,
                                   *point_y,
                                   *d);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x.value,
                                 &line->p1.y.value,
                                 &line->p2.x.value,
                                 &line->p2.y.value,
                                 &point->x.value,
                                 &point->y.value,
                                 &d->value);
    }
};

struct AngleBetweenLines : gcs::Constraint {
    gcs::g2d::Line* line1;
    gcs::g2d::Line* line2;
    gcs::Variable* angle;

    AngleBetweenLines(gcs::g2d::Line& line1,
                      gcs::g2d::Line& line2,
                      gcs::Variable& angle)
        : line1{&line1}, line2{&line2}, angle{&angle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 9;

        template <typename T>
        bool operator()(const T* line1_p1_x,
                        const T* line1_p1_y,
                        const T* line1_p2_x,
                        const T* line1_p2_y,
                        const T* line2_p1_x,
                        const T* line2_p1_y,
                        const T* line2_p2_x,
                        const T* line2_p2_y,
                        const T* angle,
                        T* r) const {
            *r = angle_point_4(*line1_p1_x,
                               *line1_p1_y,
                               *line1_p2_x,
                               *line1_p2_y,
                               *line2_p1_x,
                               *line2_p1_y,
                               *line2_p2_x,
                               *line2_p2_y,
                               *angle);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line1->p1.x.value,
                                 &line1->p1.y.value,
                                 &line1->p2.x.value,
                                 &line1->p2.y.value,
                                 &line2->p1.x.value,
                                 &line2->p1.y.value,
                                 &line2->p2.x.value,
                                 &line2->p2.y.value,
                                 &angle->value);
    }
};

struct AngleThreePoints : gcs::Constraint {
    gcs::g2d::Point* p1;
    gcs::g2d::Point* p2;
    gcs::g2d::Point* p3;
    gcs::Variable* angle;

    AngleThreePoints(gcs::g2d::Point& p1,
                     gcs::g2d::Point& p2,
                     gcs::g2d::Point& p3,
                     gcs::Variable& angle)
        : p1{&p1}, p2{&p2}, p3{&p3}, angle{&angle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 7;

        template <typename T>
        bool operator()(const T* p1_x,
                        const T* p1_y,
                        const T* p2_x,
                        const T* p2_y,
                        const T* p3_x,
                        const T* p3_y,
                        const T* angle,
                        T* r) const {
            *r =
                angle_point_3(*p1_x, *p1_y, *p2_x, *p2_y, *p3_x, *p3_y, *angle);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &p1->x.value,
                                 &p1->y.value,
                                 &p2->x.value,
                                 &p2->y.value,
                                 &p3->x.value,
                                 &p3->y.value,
                                 &angle->value);
    }
};

struct AngleOfLine : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::Variable* angle;

    AngleOfLine(gcs::g2d::Line& line, gcs::Variable& angle)
        : line{&line}, angle{&angle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template <typename T>
        bool operator()(const T* line_p1_x,
                        const T* line_p1_y,
                        const T* line_p2_x,
                        const T* line_p2_y,
                        const T* angle,
                        T* r) const {
            *r = angle_point_2(
                *line_p1_x, *line_p1_y, *line_p2_x, *line_p2_y, *angle);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x.value,
                                 &line->p1.y.value,
                                 &line->p2.x.value,
                                 &line->p2.y.value,
                                 &angle->value);
    }
};

struct PointOnCircle : gcs::Constraint {
    gcs::g2d::Point* point;
    gcs::g2d::Circle* circle;

    PointOnCircle(gcs::g2d::Point& point, gcs::g2d::Circle& circle)
        : point{&point}, circle{&circle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template <typename T>
        bool operator()(const T* point_x,
                        const T* point_y,
                        const T* circle_center_x,
                        const T* circle_center_y,
                        const T* circle_radius,
                        T* r) const {
            *r = point_on_circle(*point_x,
                                 *point_y,
                                 *circle_center_x,
                                 *circle_center_y,
                                 *circle_radius);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &point->x.value,
                                 &point->y.value,
                                 &circle->center.x.value,
                                 &circle->center.y.value,
                                 &circle->radius.value);
    }
};

struct TangentLineCircle : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::g2d::Circle* circle;

    TangentLineCircle(gcs::g2d::Line& line, gcs::g2d::Circle& circle)
        : line{&line}, circle{&circle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 7;

        template <typename T>
        bool operator()(const T* line_p1_x,
                        const T* line_p1_y,
                        const T* line_p2_x,
                        const T* line_p2_y,
                        const T* circle_center_x,
                        const T* circle_center_y,
                        const T* circle_radius,
                        T* r) const {
            *r = tangent_line_circle(*line_p1_x,
                                     *line_p1_y,
                                     *line_p2_x,
                                     *line_p2_y,
                                     *circle_center_x,
                                     *circle_center_y,
                                     *circle_radius);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x.value,
                                 &line->p1.y.value,
                                 &line->p2.x.value,
                                 &line->p2.y.value,
                                 &circle->center.x.value,
                                 &circle->center.y.value,
                                 &circle->radius.value);
    }
};

struct TangentCircles : gcs::Constraint {
    gcs::g2d::Circle* c1;
    gcs::g2d::Circle* c2;

    TangentCircles(gcs::g2d::Circle& c1, gcs::g2d::Circle& c2)
        : c1{&c1}, c2{&c2} {}

    struct Functor_0 {
        static const metal::int_ num_params = 6;

        template <typename T>
        bool operator()(const T* c1_center_x,
                        const T* c1_center_y,
                        const T* c1_radius,
                        const T* c2_center_x,
                        const T* c2_center_y,
                        const T* c2_radius,
                        T* r) const {
            *r = tangent_circles(*c1_center_x,
                                 *c1_center_y,
                                 *c1_radius,
                                 *c2_center_x,
                                 *c2_center_y,
                                 *c2_radius);
            return true;
        }
    };

    void add_to_problem(ceres::Problem& problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &c1->center.x.value,
                                 &c1->center.y.value,
                                 &c1->radius.value,
                                 &c2->center.x.value,
                                 &c2->center.y.value,
                                 &c2->radius.value);
    }
};

}  // namespace g2d

}  // namespace gcs

#endif  // GCS_G2D_CONSTRAINTS