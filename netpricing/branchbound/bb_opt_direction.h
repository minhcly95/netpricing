#pragma once

enum bb_opt_direction {
	Minimize,
	Maximize
};

template <bb_opt_direction opt_dir>
bool is_better_obj(double new_obj, double old_obj);

template <>
inline bool is_better_obj<Minimize>(double new_obj, double old_obj)
{
	return new_obj < old_obj;
}

template <>
inline bool is_better_obj<Maximize>(double new_obj, double old_obj)
{
	return new_obj > old_obj;
}

template <bb_opt_direction opt_dir, typename node_type>
struct bb_better {
	bool operator()(node_type* lhs, node_type* rhs) const {
		return is_better_obj<opt_dir>(lhs->get_bound(), rhs->get_bound());
	}
};
