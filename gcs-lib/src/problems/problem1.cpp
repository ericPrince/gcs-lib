#include <gcs/problems/problem1.h>
#include <gcs/solver/solve_elements.h>

#include <cassert>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

namespace problems {

Problem problem1() {
    Problem problem;

    // clang-format off
    std::string var_names[] = {
        "p0.x",
        "p0.y",
        "p1.x",
        "p1.y",
        "p2.x",
        "p2.y",
        "p3.x",
        "p3.y",
        "c1.p.x",
        "c1.p.y",
        "c1.r",
        "L1.p1.x",
        "L1.p1.y",
        "L1.p2.x",
        "L1.p2.y",
        "d1",
        "a1",
        "dx",
        "dy"
    };

    std::map<std::string, std::vector<std::string>> eqn_map = {
        { "f.01", { "p0.x" } }, // f1  = g2d.SetVar('f1', p0.x, 0.0)
        { "f.02", { "p0.y" } }, // f2  = g2d.SetVar('f2', p0.y, 0.0)
        { "f.03", { "c1.r" } }, // f3  = g2d.SetVar('f3', c1.r, r0)
        { "f.04", { "d1" } }, // f4  = g2d.SetVar('f4', d1, d)
        { "f.05", { "a1" } }, // f5  = g2d.SetVar('f5', a1, a)
        { "f.06", { "p0.x", "c1.p.x" } }, // f67 = g2d.CoincidentPoint2('f67', p0, c1.p)
        { "f.07", { "p0.y", "c1.p.y" } },
        { "f.08", { "p0.x", "p1.x", "dx" } }, // f8  = g2d.HorzDist('f8', p0, p1, dx)
        { "f.09", { "p0.y", "p1.y", "dy" } }, // f9  = g2d.VertDist('f9', p0, p1, dy)
        { "f.10", { "p1.x", "p1.y", "p3.x", "p3.y", "p2.x", "p2.y", "a1" } }, // f10 = g2d.AnglePoint3('f10', p1, p3, p2, a1)
        { "f.11", { "L1.p1.x", "L1.p1.y", "L1.p2.x", "L1.p2.y", "c1.p.x", "c1.p.y", "c1.r" } }, // f11 = g2d.TangentLineCircle('f11', L1, c1)
        { "f.12", { "p3.x", "p3.y", "c1.p.x", "c1.p.y", "c1.r" } }, // f12 = g2d.PointOnCircle('f12', p3, c1)
        { "f.13", { "L1.p1.x", "p3.x" } }, // f1314 = g2d.CoincidentPoint2('f1314', L1.p1, p3)
        { "f.14", { "L1.p1.y", "p3.y" } },
        { "f.15", { "L1.p1.x", "L1.p1.y", "L1.p2.x", "L1.p2.y", "d1" } }, // f15 = g2d.LineLength('f15', L1, d1)
        { "f.16", { "L1.p2.x", "p2.x" } }, // f1617 = g2d.CoincidentPoint2('f1617', L1.p2, p2)
        { "f.17", { "L1.p2.y", "p2.y" } },
        { "f.18", { "dx" } }, // f18 = g2d.SetVar('f18', dx, d_x)
        { "f.19", { "dy" } }, // f19 = g2d.SetVar('f19', dy, d_y)
    };
    // clang-format on

    // create vars for each name
    for (const auto& name : var_names) {
        problem.vars.emplace(name, GCS::Variable{});
        problem.var_names.emplace(&problem.vars.at(name), name);
    }

    // create eqns for each name and give the vars required for that eqn
    for (const auto& eqn : eqn_map) {
        std::unordered_set<GCS::Variable*> eqn_vars;

        for (const auto& var_name : eqn.second) {
            assert(problem.vars.find(var_name) != problem.vars.end());

            eqn_vars.insert(&problem.vars.at(var_name));
        }

        problem.eqns.emplace(eqn.first, GCS::Equation{std::move(eqn_vars)});
        problem.eqn_names.emplace(&problem.eqns.at(eqn.first), eqn.first);
    }

    // check for validity
    for (auto& var : problem.vars) {
        for (auto& eqn : var.second.equations) {
            assert(problem.eqn_names.find(eqn) != problem.eqn_names.end());
        }
    }

    return problem;
}

}  // namespace problems
