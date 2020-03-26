#pragma once

#include "../model.h"

void set_x_name(model_single& m, cplex_def::VarMatrix& x);
void set_y_name(model_single& m, cplex_def::VarMatrix& y);
void set_t_name(model_single& m, cplex_def::VarArray& t);
void set_tx_name(model_single& m, cplex_def::VarMatrix& tx);
void set_lambda_name(model_single& m, cplex_def::VarMatrix& lambda);

#define SET_VAR_NAME(m, var) set_ ## var ## _name(m, var)

#define SET_VAR_NAMES_1(m, v1) SET_VAR_NAME(m, v1)
#define SET_VAR_NAMES_2(m, v1, v2) SET_VAR_NAME(m, v2); SET_VAR_NAMES_1(m, v1)
#define SET_VAR_NAMES_3(m, v1, v2, v3) SET_VAR_NAME(m, v3); SET_VAR_NAMES_2(m, v1, v2)
#define SET_VAR_NAMES_4(m, v1, v2, v3, v4) SET_VAR_NAME(m, v4); SET_VAR_NAMES_3(m, v1, v2, v3)
#define SET_VAR_NAMES_5(m, v1, v2, v3, v4, v5) SET_VAR_NAME(m, v5); SET_VAR_NAMES_4(m, v1, v2, v3, v4)

#define GET_MACRO(_1, _2, _3, _4, _5, NAME, ...) NAME
#define SET_VAR_NAMES(m, ...) GET_MACRO(__VA_ARGS__, SET_VAR_NAMES_5, SET_VAR_NAMES_4, SET_VAR_NAMES_3, SET_VAR_NAMES_2, SET_VAR_NAMES_1)(m, __VA_ARGS__)
