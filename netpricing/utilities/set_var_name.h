#pragma once

#include "../model.h"

void set_x_name(model_single& m, cplex_def::VarMatrix& x);
void set_y_name(model_single& m, cplex_def::VarMatrix& y);
void set_t_name(model_single& m, cplex_def::VarArray& t);
void set_tx_name(model_single& m, cplex_def::VarMatrix& tx);
void set_lambda_name(model_single& m, cplex_def::VarMatrix& lambda);

void set_x_name_k(model_single& m, int k, cplex_def::VarArray& x);
void set_y_name_k(model_single& m, int k, cplex_def::VarArray& y);
void set_tx_name_k(model_single& m, int k, cplex_def::VarArray& tx);
void set_lambda_name_k(model_single& m, int k, cplex_def::VarArray& lambda);

void set_x_name_k(model_single* m, int k, cplex_def::VarArray& x);
void set_y_name_k(model_single* m, int k, cplex_def::VarArray& y);
void set_tx_name_k(model_single* m, int k, cplex_def::VarArray& tx);
void set_lambda_name_k(model_single* m, int k, cplex_def::VarArray& lambda);
void set_z_name_k(model_single* m, int k, cplex_def::VarArray& z);
void set_tz_name_k(model_single* m, int k, cplex_def::VarMatrix& tz);

#define GET_MACRO(_1, _2, _3, _4, _5, NAME, ...) NAME

#define SET_VAR_NAME(m, var) set_ ## var ## _name(m, var)

#define SET_VAR_NAMES_1(m, v1) SET_VAR_NAME(m, v1)
#define SET_VAR_NAMES_2(m, v1, v2) SET_VAR_NAME(m, v2); SET_VAR_NAMES_1(m, v1)
#define SET_VAR_NAMES_3(m, v1, v2, v3) SET_VAR_NAME(m, v3); SET_VAR_NAMES_2(m, v1, v2)
#define SET_VAR_NAMES_4(m, v1, v2, v3, v4) SET_VAR_NAME(m, v4); SET_VAR_NAMES_3(m, v1, v2, v3)
#define SET_VAR_NAMES_5(m, v1, v2, v3, v4, v5) SET_VAR_NAME(m, v5); SET_VAR_NAMES_4(m, v1, v2, v3, v4)

#define SET_VAR_NAMES(m, ...) GET_MACRO(__VA_ARGS__, SET_VAR_NAMES_5, SET_VAR_NAMES_4, SET_VAR_NAMES_3, SET_VAR_NAMES_2, SET_VAR_NAMES_1)(m, __VA_ARGS__)

#define SET_VAR_NAME_K(m, k, var) set_ ## var ## _name_k(m, k, var)

#define SET_VAR_NAMES_K_1(m, k, v1) SET_VAR_NAME_K(m, k, v1)
#define SET_VAR_NAMES_K_2(m, k, v1, v2) SET_VAR_NAME_K(m, k, v2); SET_VAR_NAMES_K_1(m, k, v1)
#define SET_VAR_NAMES_K_3(m, k, v1, v2, v3) SET_VAR_NAME_K(m, k, v3); SET_VAR_NAMES_K_2(m, k, v1, v2)
#define SET_VAR_NAMES_K_4(m, k, v1, v2, v3, v4) SET_VAR_NAME_K(m, k, v4); SET_VAR_NAMES_K_3(m, k, v1, v2, v3)
#define SET_VAR_NAMES_K_5(m, k, v1, v2, v3, v4, v5) SET_VAR_NAME_K(m, k, v5); SET_VAR_NAMES_K_4(m, k, v1, v2, v3, v4)

#define SET_VAR_NAMES_K(m, k, ...) GET_MACRO(__VA_ARGS__, SET_VAR_NAMES_K_5, SET_VAR_NAMES_K_4, SET_VAR_NAMES_K_3, SET_VAR_NAMES_K_2, SET_VAR_NAMES_K_1)(m, k, __VA_ARGS__)
