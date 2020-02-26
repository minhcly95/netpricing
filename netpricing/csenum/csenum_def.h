#pragma once

enum csenum_branch_dir : bool {
	PRIMAL,
	DUAL
};

struct csenum_coor {
	int k;
	int a;
};

struct csenum_branch {
	int k;
	int a;
	csenum_branch_dir dir;
};