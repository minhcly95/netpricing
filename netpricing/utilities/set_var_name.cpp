#include "set_var_name.h"

#include "../macros.h"

#include <sstream>

void set_x_name(model_single& m, cplex_def::VarMatrix& x)
{
	LOOP(k, m.K) set_x_name_k(m, k, x[k]);
}
void set_y_name(model_single& m, cplex_def::VarMatrix& y)
{
	LOOP(k, m.K) set_y_name_k(m, k, y[k]);
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
	LOOP(k, m.K) set_tx_name_k(m, k, tx[k]);
}
void set_lambda_name(model_single& m, cplex_def::VarMatrix& lambda)
{
	LOOP(k, m.K) set_lambda_name_k(m, k, lambda[k]);
}

void set_x_name_k(model_single& m, int k, cplex_def::VarArray& x)
{
	set_x_name_k(&m, k, x);
}
void set_y_name_k(model_single& m, int k, cplex_def::VarArray& y)
{
	set_y_name_k(&m, k, y);
}
void set_tx_name_k(model_single& m, int k, cplex_def::VarArray& tx)
{
	set_tx_name_k(&m, k, tx);
}
void set_lambda_name_k(model_single& m, int k, cplex_def::VarArray& lambda)
{
	set_lambda_name_k(&m, k, lambda);
}

void set_x_name_k(model_single* m, int k, cplex_def::VarArray& x)
{
	LOOP(a, m->A1) {
		SRC_DST_FROM_A1(m->prob, a);
		char name[50];
		sprintf(name, "x[%d,%d->%d]", k, src, dst);
		x[a].setName(name);
	}
}
void set_y_name_k(model_single* m, int k, cplex_def::VarArray& y)
{
	LOOP(a, m->A2) {
		SRC_DST_FROM_A2(m->prob, a);
		char name[50];
		sprintf(name, "y[%d,%d->%d]", k, src, dst);
		y[a].setName(name);
	}
}
void set_tx_name_k(model_single* m, int k, cplex_def::VarArray& tx)
{
	LOOP(a, m->A1) {
		SRC_DST_FROM_A1(m->prob, a);
		char name[50];
		sprintf(name, "tx[%d,%d->%d]", k, src, dst);
		tx[a].setName(name);
	}
}
void set_lambda_name_k(model_single* m, int k, cplex_def::VarArray& lambda)
{
	LOOP(i, m->V) {
		char name[50];
		sprintf(name, "lbd[%d,%d]", k, i);
		lambda[i].setName(name);
	}
}

void set_z_name_k(model_single* m, int k, cplex_def::VarArray& z)
{
	LOOP(p, z.getSize()) {
		char name[50];
		sprintf(name, "z[%d,%d]", k, p);
		z[p].setName(name);
	}
}

void set_tz_name_k(model_single* m, int k, cplex_def::VarMatrix& tz)
{
	LOOP(p, tz.getSize()) LOOP(a, m->A1) {
		SRC_DST_FROM_A1(m->prob, a);
		char name[50];
		sprintf(name, "tz[%d,%d,%d->%d]", k, p, src, dst);
		tz[p][a].setName(name);
	}
}
