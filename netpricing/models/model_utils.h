#pragma once

#include "model.h"

model_base::NumArray get_values(IloCplex& cplex, const model_base::NumVarArray& vars);
model_base::NumMatrix get_values(IloCplex& cplex, const model_base::NumVarMatrix& vars);
void clean_up(model_base::NumVarMatrix& vars);
void clean_up(model_base::NumMatrix& vals);

solution fetch_solution_from_z_t(const model_single& m, model_base::NumMatrix& zvals, model_base::NumArray& tvals);

solution fetch_solution_from_xy_t(const model_single& m, model_base::NumMatrix& xvals, model_base::NumMatrix& yvals, model_base::NumArray& tvals);
