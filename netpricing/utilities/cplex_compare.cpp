#include "cplex_compare.h"

bool operator<(const IloNumVar& v1, const IloNumVar& v2)
{
	return v1.getId() < v2.getId();
}
