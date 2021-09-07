#include <ceres/ceres.h>
#include <metal.h>

#include <vector>

#include "gcs/core/core.h"

gcs::g2d::Point : gcs::Geometry {
    gcs::Variable x;
    gcs::Variable y;

    gcs::g2d::Point(gcs::Variable x, gcs::Variable y) : x{x}, y{y} {}

    std::vector<gcs::Variable*> get_variables() const {
        return { &x, &y }
    }
};

gcs::g2d::Line : gcs::Geometry {
    gcs::g2d::Point p1;
    gcs::g2d::Point p2;

    gcs::g2d::Line(gcs::g2d::Point p1, gcs::g2d::Point p2) : p1{p1}, p2{p2} {}

    std::vector<gcs::Variable*> get_variables() const {
        return { &p1.x, &p1.y, &p2.x, &p2.y }
    }
};

gcs::g2d::Circle : gcs::Geometry {
    gcs::g2d::Point center;
    gcs::Variable radius;

    gcs::g2d::Circle(gcs::g2d::Point center, gcs::Variable radius)
        : center{center}, radius{radius} {}

    std::vector<gcs::Variable*> get_variables() const {
        return { &center.x, &center.y, &radius }
    }
};

gcs::SetConstant : gcs::Constraint {
    gcs::Variable* var;
    double value;

    gcs::SetConstant(gcs::Variable & var, double value)
        : var{&var}, value{value} {}

    struct Functor_0 {
        static const metal::int_ num_params = 1;
        double value;

        template typename<T> bool operator()(const T* var, T* r) const {
            *r = equate(*var, value);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(
            gcs::create_scalar_autodiff(new Functor_0{value}), nullptr, &var);
    }
};

gcs::Equate : gcs::Constraint {
    gcs::Variable* v1;
    gcs::Variable* v2;

    gcs::Equate(gcs::Variable & v1, gcs::Variable & v2) : v1{&v1}, v2{&v2} {}

    struct Functor_0 {
        static const metal::int_ num_params = 2;

        template typename<T> bool operator()(const T* v1,
                                             const T* v2,
                                             T* r) const {
            *r = equate(*v1, *v2);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(
            gcs::create_scalar_autodiff(new Functor_0{}), nullptr, &v1, &v2);
    }
};

gcs::Difference : gcs::Constraint {
    gcs::Variable* v1;
    gcs::Variable* v2;
    gcs::Variable* diff;

    gcs::Difference(
        gcs::Variable & v1, gcs::Variable & v2, gcs::Variable & diff)
        : v1{&v1}, v2{&v2}, diff{&diff} {}

    struct Functor_0 {
        static const metal::int_ num_params = 3;

        template typename<T> bool operator()(const T* v1,
                                             const T* v2,
                                             const T* diff,
                                             T* r) const {
            *r = difference(*v1, *v2, *diff);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &v1,
                                 &v2,
                                 &diff);
    }
};

gcs::g2d::PointOnLine : gcs::Constraint {
    gcs::g2d::Point* point;
    gcs::g2d::Line* line;

    gcs::g2d::PointOnLine(gcs::g2d::Point & point, gcs::g2d::Line & line)
        : point{&point}, line{&line} {}

    struct Functor_0 {
        static const metal::int_ num_params = 6;

        template typename<T> bool operator()(const T* point_x,
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

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &point->x,
                                 &point->y,
                                 &line->p1.x,
                                 &line->p1.y,
                                 &line->p2.x,
                                 &line->p2.y);
    }
};

gcs::g2d::CoincidentPoints : gcs::Constraint {
    gcs::g2d::Point* p1;
    gcs::g2d::Point* p2;

    gcs::g2d::CoincidentPoints(gcs::g2d::Point & p1, gcs::g2d::Point & p2)
        : p1{&p1}, p2{&p2} {}

    struct Functor_0 {
        static const metal::int_ num_params = 2;

        template typename<T> bool operator()(const T* p1_x,
                                             const T* p2_x,
                                             T* r) const {
            *r = equate(*p1_x, *p2_x);
            return true;
        }
    };
    struct Functor_1 {
        static const metal::int_ num_params = 2;

        template typename<T> bool operator()(const T* p1_y,
                                             const T* p2_y,
                                             T* r) const {
            *r = equate(*p1_y, *p2_y);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &p1->x,
                                 &p2->x);
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_1{}),
                                 nullptr,
                                 &p1->y,
                                 &p2->y);
    }
};

gcs::g2d::DistancePoints : gcs::Constraint {
    gcs::g2d::Point* p1;
    gcs::g2d::Point* p2;
    gcs::Variable* d;

    gcs::g2d::DistancePoints(
        gcs::g2d::Point & p1, gcs::g2d::Point & p2, gcs::Variable & d)
        : p1{&p1}, p2{&p2}, d{&d} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template typename<T> bool operator()(const T* p1_x,
                                             const T* p1_y,
                                             const T* p2_x,
                                             const T* p2_y,
                                             const T* d,
                                             T* r) const {
            *r = distance(*p1_x, *p1_y, *p2_x, *p2_y, *d);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &p1->x,
                                 &p1->y,
                                 &p2->x,
                                 &p2->y,
                                 &d);
    }
};

gcs::g2d::LineLength : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::Variable* d;

    gcs::g2d::LineLength(gcs::g2d::Line & line, gcs::Variable & d)
        : line{&line}, d{&d} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template typename<T> bool operator()(const T* line_p1_x,
                                             const T* line_p1_y,
                                             const T* line_p2_x,
                                             const T* line_p2_y,
                                             const T* d,
                                             T* r) const {
            *r = distance(*line_p1_x, *line_p1_y, *line_p2_x, *line_p2_y, *d);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x,
                                 &line->p1.y,
                                 &line->p2.x,
                                 &line->p2.y,
                                 &d);
    }
};

gcs::g2d::OffsetLinePoint : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::g2d::Point* point;
    gcs::Variable* d;

    gcs::g2d::OffsetLinePoint(
        gcs::g2d::Line & line, gcs::g2d::Point & point, gcs::Variable & d)
        : line{&line}, point{&point}, d{&d} {}

    struct Functor_0 {
        static const metal::int_ num_params = 7;

        template typename<T> bool operator()(const T* line_p1_x,
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

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x,
                                 &line->p1.y,
                                 &line->p2.x,
                                 &line->p2.y,
                                 &point->x,
                                 &point->y,
                                 &d);
    }
};

gcs::g2d::AngleBetweenLines : gcs::Constraint {
    gcs::g2d::Line* line1;
    gcs::g2d::Line* line2;
    gcs::Variable* angle;

    gcs::g2d::AngleBetweenLines(
        gcs::g2d::Line & line1, gcs::g2d::Line & line2, gcs::Variable & angle)
        : line1{&line1}, line2{&line2}, angle{&angle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 9;

        template typename<T> bool operator()(const T* line1_p1_x,
                                             const T* line1_p1_y,
                                             const T* line1_p2_x,
                                             const T* line1_p2_y,
                                             const T* line2_p1_x,
                                             const T* line2_p1_y,
                                             const T* line2_p2_x,
                                             const T* line2_p2_y,
                                             const T* angle,
                                             T* r) const {
            *r = angle_points_4(*line1_p1_x,
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

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line1->p1.x,
                                 &line1->p1.y,
                                 &line1->p2.x,
                                 &line1->p2.y,
                                 &line2->p1.x,
                                 &line2->p1.y,
                                 &line2->p2.x,
                                 &line2->p2.y,
                                 &angle);
    }
};

gcs::g2d::AngleThreePoints : gcs::Constraint {
    gcs::g2d::Point* p1;
    gcs::g2d::Point* p2;
    gcs::g2d::Point* p3;
    gcs::Variable* angle;

    gcs::g2d::AngleThreePoints(gcs::g2d::Point & p1,
                               gcs::g2d::Point & p2,
                               gcs::g2d::Point & p3,
                               gcs::Variable & angle)
        : p1{&p1}, p2{&p2}, p3{&p3}, angle{&angle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 7;

        template typename<T> bool operator()(const T* p1_x,
                                             const T* p1_y,
                                             const T* p2_x,
                                             const T* p2_y,
                                             const T* p3_x,
                                             const T* p3_y,
                                             const T* angle,
                                             T* r) const {
            *r = angle_points_3(
                *p1_x, *p1_y, *p2_x, *p2_y, *p3_x, *p3_y, *angle);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &p1->x,
                                 &p1->y,
                                 &p2->x,
                                 &p2->y,
                                 &p3->x,
                                 &p3->y,
                                 &angle);
    }
};

gcs::g2d::AngleOfLine : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::Variable* angle;

    gcs::g2d::AngleOfLine(gcs::g2d::Line & line, gcs::Variable & angle)
        : line{&line}, angle{&angle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template typename<T> bool operator()(const T* line_p1_x,
                                             const T* line_p1_y,
                                             const T* line_p2_x,
                                             const T* line_p2_y,
                                             const T* angle,
                                             T* r) const {
            *r = angle_points_2(
                *line_p1_x, *line_p1_y, *line_p2_x, *line_p2_y, *angle);
            return true;
        }
    };

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x,
                                 &line->p1.y,
                                 &line->p2.x,
                                 &line->p2.y,
                                 &angle);
    }
};

gcs::g2d::PointOnCircle : gcs::Constraint {
    gcs::g2d::Point* point;
    gcs::g2d::Circle* circle;

    gcs::g2d::PointOnCircle(gcs::g2d::Point & point, gcs::g2d::Circle & circle)
        : point{&point}, circle{&circle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 5;

        template typename<T> bool operator()(const T* point_x,
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

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &point->x,
                                 &point->y,
                                 &circle->center.x,
                                 &circle->center.y,
                                 &circle->radius);
    }
};

gcs::g2d::TangentLineCircle : gcs::Constraint {
    gcs::g2d::Line* line;
    gcs::g2d::Circle* circle;

    gcs::g2d::TangentLineCircle(gcs::g2d::Line & line,
                                gcs::g2d::Circle & circle)
        : line{&line}, circle{&circle} {}

    struct Functor_0 {
        static const metal::int_ num_params = 7;

        template typename<T> bool operator()(const T* line_p1_x,
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

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &line->p1.x,
                                 &line->p1.y,
                                 &line->p2.x,
                                 &line->p2.y,
                                 &circle->center.x,
                                 &circle->center.y,
                                 &circle->radius);
    }
};

gcs::g2d::TangentCircles : gcs::Constraint {
    gcs::g2d::Circle* c1;
    gcs::g2d::Circle* c2;

    gcs::g2d::TangentCircles(gcs::g2d::Circle & c1, gcs::g2d::Circle & c2)
        : c1{&c1}, c2{&c2} {}

    struct Functor_0 {
        static const metal::int_ num_params = 6;

        template typename<T> bool operator()(const T* c1_center_x,
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

    void add_to_problem(ceres::Problem & problem) {
        problem.AddResidualBlock(gcs::create_scalar_autodiff(new Functor_0{}),
                                 nullptr,
                                 &c1->center.x,
                                 &c1->center.y,
                                 &c1->radius,
                                 &c2->center.x,
                                 &c2->center.y,
                                 &c2->radius);
    }
};