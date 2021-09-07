from typing import *
import itertools as it

from pydantic import BaseModel, validator



class GeometryReference(BaseModel):
    type: str
    name: str


class GeometryDefinition(BaseModel):
    classname: str
    namespace: str = ""
    variables: List[str] = []
    geoms: List[GeometryReference] = []

    @property
    def fullname(self):
        ns = self.namespace.replace('.', '::')
        return f'{ns}::{self.classname}'

    def get_all_vars(self, geom_types) -> List[str]:
        return sum(([f"{gref.name}.{var}" for var in geom_types[gref.type].get_all_vars(geom_types)] for gref in self.geoms), []) + self.variables

    def make_struct(self, geom_types) -> str:
        param_names = (
            [(geom_types[gref.type].fullname, gref.name) for gref in self.geoms]
            + [('gcs::Variable', var) for var in self.variables]
        )

        return '\n'.join(
            [f'{self.fullname} : gcs::Geometry {{']
            + [f"    {type_} {name};" for type_, name in param_names]
            + [
                '',
                f'    {self.fullname}(' + ', '.join([f'{type_} {name}' for type_, name in param_names]) + ')',
                '        : ' + ', '.join([f'{name}{{{name}}}' for _, name in param_names]) + ' {}',
                '',
                '    std::vector<gcs::Variable*> get_variables() const {',
                '        return {' + ', '.join([f'&{var}' for var in self.get_all_vars(geom_types)]) + '}',
                '    }'
            ]
            + ['};']
        ) 


class FtorArg(BaseModel):
    name: str


class EquationUsage(BaseModel):
    funcname: str
    variables: Union[Literal["all"], List[str]] = "all"
    ftor_args: List[FtorArg] = []

    def get_variables(self, constraint: 'Constraint', geom_types):
        return constraint.get_all_vars(geom_types) if self.variables == 'all' else self.variables

    def make_functor(self, constraint: 'Constraint', geom_types, functor_suffix=None) -> str:
        variables = [var.replace(".", "_") for var in self.get_variables(constraint, geom_types)]

        if functor_suffix is None:
            functor_suffix = self.funcname

        return '\n'.join([
            f'    struct Functor_{functor_suffix} {{',
            f'        static const metal::int_ num_params = {len(variables)};',
        ] + [
            f'        double {arg.name};' for arg in self.ftor_args
        ] + [
            '',
            f'        template typename<T>',
            '        bool operator()(' + ''.join((f'const T* {var}, ' for var in variables)) + 'T* r) const {',
            f'            *r = {self.funcname}(' + ', '.join([f'*{var}' for var in variables] + [arg.name for arg in self.ftor_args]) + ');',
            '            return true;',
            '        }',
            '    };',
        ])

    def make_residual_statement(self, constraint: 'Constraint', functor_suffix, geom_types):
        return '\n'.join(
            [
                '        problem.AddResidualBlock(',
                f'            gcs::create_scalar_autodiff(new Functor_{functor_suffix}{{' + ', '.join([arg.name for arg in self.ftor_args]) + '}),',
                '            nullptr' 
                    + ''.join([
                        ',\n            ' + f'&{var.replace(".", "->", 1)}' 
                        for var in self.get_variables(constraint, geom_types)
                    ]),
                '        );',
            ]
        )


class ConstraintDefinition(BaseModel):
    classname: str
    namespace: str = ""
    variables: List[str] = []
    geoms: List[GeometryReference] = []
    equations: List[EquationUsage]

    @property
    def fullname(self):
        ns = self.namespace.replace('.', '::')
        return f'{ns}::{self.classname}'

    def get_all_vars(self, geom_types) -> List[str]:
        return sum(([f"{gref.name}.{var}" for var in geom_types[gref.type].get_all_vars(geom_types)] for gref in self.geoms), []) + self.variables

    def make_struct(self, geom_types):
        param_names = (
            [(geom_types[gref.type].fullname, gref.name) for gref in self.geoms]
            + [('gcs::Variable', var) for var in self.variables]
            + [('double', arg.name) for eqn in self.equations for arg in eqn.ftor_args]
        )

        return '\n'.join(
            [f'{self.fullname} : gcs::Constraint {{']
            + [f"    {type_}* {name};" if type_ != 'double' else f'    double {name};' for type_, name in param_names]
            + [
                '',
                f'    {self.fullname}(' + ', '.join([f'{type_}& {name}' if type_ != 'double' else f'double {name}' for type_, name in param_names]) + ')',
                '        : ' + ', '.join([f'{name}{{&{name}}}' if type_ != 'double' else f'{name}{{{name}}}' for type_, name in param_names]) + ' {}',
                '',
            ]
            + [eqn.make_functor(self, geom_types, str(i)) for i, eqn in enumerate(self.equations)]
            + [
                '',
                '    void add_to_problem(ceres::Problem& problem) {'
            ]
            + [
                eqn.make_residual_statement(self, str(i), geom_types)
                for i, eqn in enumerate(self.equations)
            ]
            + [
                '    }',
                '};',
            ]
        ) 


class GcsDefinitions(BaseModel):
    geometry_definitions: List[GeometryDefinition]
    constraint_definitions: List[ConstraintDefinition]

    def get_geom_types(self) -> Dict[str, GeometryDefinition]:
        return {f'{geom.namespace}.{geom.classname}': geom for geom in self.geometry_definitions}
    
    def make_structs(self) -> str:
        geom_types = self.get_geom_types()

        return '\n\n'.join(
            [
                '#include <vector>',
                '#include <ceres/ceres.h>',
                '#include <metal.h>',
                '#include "gcs/core/core.h"',
            ]
            + [
                geom.make_struct(geom_types) 
                for geom in it.chain(self.geometry_definitions, self.constraint_definitions)
            ]
        )


def main():
    import yaml

    with open('gcs_definitions.yml') as fp:
        gcs_data = yaml.load(fp, Loader=yaml.SafeLoader)

    gcs_defs = GcsDefinitions.parse_obj(gcs_data)
    # print(gcs_defs.make_structs())

    with open('gcs_definitions.cpp', 'w') as fp:
        fp.write(gcs_defs.make_structs())


if __name__ == '__main__':
    main()
