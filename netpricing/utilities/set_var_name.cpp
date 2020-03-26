#include "set_var_name.h"

#include "../macros.h"

#include <sstream>

void set_x_name(model_single& m, cplex_def::VarMatrix& x)
{
	LOOP(k, m.K) LOOP(a, m.A1) {
		SRC_DST_FROM_A1(m.prob, a);
		char name[50];
		sprintf(name, "x[%d,%d->%d]", k, src, dst);
		x[k][a].setName(name);
	}
}

void set_y_name(model_single& m, cplex_def::VarMatrix& y)
{
	LOOP(k, m.K) LOOP(a, m.A2) {
		SRC_DST_FROM_A2(m.prob, a);
		char name[50];
		sprintf(name, "y[%d,%d->%d]", k, src, dst);
		y[k][a].setName(name);
	}
}

void set_t_name(model_single& m, cplex_def::VarArray& t)
{
	LOOP(a, m.A1) {
		SRC_DST_FROM_A1(m.prob, a);
		char name[50];
		sprintf(name, "t[%d->%d]", src, dst);
		t[a].setName(name);
	}
}

void set_tx_name(model_single& m, cplex_def::VarMatrix& tx)
{
	LOOP(k, m.K) LOOP(a, m.A1) {
		SRC_DST_FROM_A1(m.prob, a);
		char name[50];
		sprintf(name, "tx[%d,%d->%d]", k, src, dst);
		tx[k][a].setName(name);
	}
}

void set_lambda_name(model_single& m, cplex_def::VarMatrix& lambda)
{
	LOOP(k, m.K) LOOP(i, m.V) {
		char name[50];
		sprintf(name, "lbd[%d,%d]", k, i);
		lambda[k][i].setName(name);
	}
}
