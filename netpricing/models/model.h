#pragma once

#include <ilcplex/ilocplex.h>

struct model {
	IloModel cplex_model;

	model(IloEnv& env) : cplex_model(env) {}
};
