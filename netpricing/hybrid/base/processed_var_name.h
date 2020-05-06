#pragma once

#include "../../utilities/set_var_name.h"
#include "preprocessor.h"

void set_x_name_k(preprocess_info& info, int k, cplex_def::VarArray& x);
void set_y_name_k(preprocess_info& info, int k, cplex_def::VarArray& y);
void set_tx_name_k(preprocess_info& info, int k, cplex_def::VarArray& tx);
void set_lambda_name_k(preprocess_info& info, int k, cplex_def::VarArray& lambda);
