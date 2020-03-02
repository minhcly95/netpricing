#include "csenum_node.h"
#include "../macros.h"

using namespace std;

csenum_node* csenum_node::clone() const
{
	return new csenum_node(*this);
}

double csenum_node::get_bound() const
{
	return bound;
}

const std::vector<csenum_coor>& csenum_node::get_candidates() const
{
	return candidates;
}
