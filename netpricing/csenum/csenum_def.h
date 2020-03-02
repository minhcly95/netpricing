#pragma once

#include <tuple>

enum csenum_branch_dir : bool {
	PRIMAL,
	DUAL
};

struct csenum_coor {
	int k;
	int a;

	friend bool operator<(const csenum_coor& lhs, const csenum_coor& rhs)
	{
		return std::tie(lhs.k, lhs.a) < std::tie(rhs.k, rhs.a);
	}
};
