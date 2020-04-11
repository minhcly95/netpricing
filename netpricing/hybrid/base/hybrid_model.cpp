#include "hybrid_model.h"

#include "formulation.h"
#include "../../macros.h"
#include "../../utilities/set_var_name.h"

#include <iostream>

using namespace std;

struct hybrid_callback : public IloCplex::Callback::Function {
	formulation* form;

	hybrid_callback(formulation* form)
		: form(form) {
	}

	virtual void invoke(const IloCplex::Callback::Context& context) override {
		form->invoke_callback(context);
	}
};

hybrid_model::hybrid_model(IloEnv& env, const problem& _prob) :
	model_with_generic_callbacks(env), model_single(_prob),
	t(env, A1, 0, IloInfinity)
{
	SET_VAR_NAMES(*this, t);
	obj = IloMaximize(env);

	cplex_model.add(obj);
	cplex_model.add(t);
}

hybrid_model::~hybrid_model()
{
	for (auto f : all_formulations) {
		delete f;
	}
}

void hybrid_model::formulate()
{
	all_formulations = assign_formulations();
	LOOP(k, K) {
		all_formulations[k]->formulate(this, k);
	}
}

bool hybrid_model::solve_impl()
{
	cout << "Formulating..." << endl;
	formulate();
	cout << "Formulation done" << endl << endl;

	return model_with_generic_callbacks::solve_impl();
}

solution hybrid_model::get_solution()
{
	return solution();
}

vector<pair<IloCplex::Callback::Function*, hybrid_model::ContextId>> hybrid_model::attach_callbacks()
{
	vector<pair<IloCplex::Callback::Function*, ContextId>> callbacks;

	for (formulation* f : all_formulations) {
		if (f->has_callback())
			callbacks.emplace_back(new hybrid_callback(f), CPX_CALLBACKCONTEXT_CANDIDATE);
	}

	return callbacks;
}
