#pragma once

#include "model.h"

model::NumArray get_values(IloCplex& cplex, const model::NumVarArray& vars);
model::NumMatrix get_values(IloCplex& cplex, const model::NumVarMatrix& vars);
void clean_up(model::NumVarMatrix& vars);
void clean_up(model::NumMatrix& vals);

solution fetch_solution_from_z_t(const model& m, model::NumMatrix& zvals, model::NumArray& tvals);

solution fetch_solution_from_xy_t(const model& m, model::NumMatrix& xvals, model::NumMatrix& yvals, model::NumArray& tvals);
