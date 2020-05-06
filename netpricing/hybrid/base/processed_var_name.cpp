#include "processed_var_name.h"

#include <sstream>

using namespace std;

void set_x_name_k(preprocess_info& info, int k, cplex_def::VarArray& x)
{
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		char name[50];
		sprintf(name, "x[%d,%d->%d]", k, arc.first, arc.second);
		x[a].setName(name);
	}
}
void set_y_name_k(preprocess_info& info, int k, cplex_def::VarArray& y)
{
	LOOP_INFO(a, A2) {
		auto arc = info.bimap_A2.left.at(a);
		char name[50];
		sprintf(name, "y[%d,%d->%d]", k, arc.first, arc.second);
		y[a].setName(name);
	}
}
void set_tx_name_k(preprocess_info& info, int k, cplex_def::VarArray& tx)
{
	LOOP_INFO(a, A1) {
		auto arc = info.bimap_A1.left.at(a);
		char name[50];
		sprintf(name, "tx[%d,%d->%d]", k, arc.first, arc.second);
		tx[a].setName(name);
	}
}
void set_lambda_name_k(preprocess_info& info, int k, cplex_def::VarArray& lambda)
{
	LOOP_INFO(i, V) {
		char name[50];
		sprintf(name, "lbd[%d,%d]", k, i);
		lambda[i].setName(name);
	}
}
