# gcs-lib

A library for geometric constraint solving

gcs-lib makes use of [Ceres Solver](http://ceres-solver.org/) to solve the geometric
constraints.

## Building

This project uses the [bazel](https://bazel.build/) build system. The repo also contains
a setup for developing using a [VS Code devcontainer](https://code.visualstudio.com/docs/remote/create-dev-container).

To build and run the sample problem/test:

```bash
bazel test --test_output=all //gcs:problem1_test
```

To build all components of gcs:

```bash
bazel build //gcs:all
```

## Concepts

The gcs-lib contains a few key components, including:

- An algorithm for splitting equations into smaller sets of fully constrained problems
- Base components including `Variable`, `Equation`, `EquationSet`, `Constraint`, and `Geometry`
- Implementations of common geometric constraints, as Ceres Solver residual blocks

### Equation Set Splitting

The `split` function takes in a set of equations and splits it into multiple smaller sets
that are each fully constrained. This allows smaller groups of equation sets to be solved
in turn, using a graph dependency approach, and also helps validate that a problem is valid.

A fully constrained equation set is a group of equations where the number of unknown variables
matches the number of unique equations.

### TODO

- a lot
- generate a graph of equation set dependencies from the equation set splitting results
- tie split sets into running ceres solver
- visualize geometry results
- consider adding algorithms from open cascade
- add 3d geometry and constraints
- add more 2d geometry components (curves, infinite lines, arcs, ...)
