#pragma once

#include "model_cplex.h"

model_cplex::NumMatrix make_num_matrix(IloEnv env, int d1, int d2);
model_cplex::NumVarMatrix make_numvar_matrix(IloEnv env, int d1, int d2);
model_cplex::RangeMatrix make_range_matrix(IloEnv env, int d1, int d2);

model_cplex::NumArray get_values(IloCplex& cplex, const model_cplex::NumVarArray& vars);
model_cplex::NumMatrix get_values(IloCplex& cplex, const model_cplex::NumVarMatrix& vars);
void clean_up(model_cplex::NumVarMatrix& vars);
void clean_up(model_cplex::NumMatrix& vals);

solution fetch_solution_from_z_t(const model_single& m, model_cplex::NumMatrix& zvals, model_cplex::NumArray& tvals);

solution fetch_solution_from_xy_t(const model_single& m, model_cplex::NumMatrix& xvals, model_cplex::NumMatrix& yvals, model_cplex::NumArray& tvals);
