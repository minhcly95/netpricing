#pragma once

#include "csenum_def.h"
#include "../typedef.h"
#include "../branchbound/bb_node.h"

#include <vector>

struct csenum_node : public bb_node<csenum_coor> {
	std::vector<double> primal_objs;
	double dual_obj;
	double bound;

	std::vector<std::vector<int>> arcs;
	std::vector<cost_type> tolls;
	std::vector<std::vector<bool>> slack_map;
	std::vector<csenum_coor> candidates;

	// Inherited via bb_node
	virtual csenum_node* clone() const override;
	virtual double get_bound() const override;
	virtual const std::vector<csenum_coor>& get_candidates() const override;
};
