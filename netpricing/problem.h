#pragma once

#include <vector>

#include <boost/graph/adjacency_list.hpp>

#include "commodity.h"
#include "typedef.h"

struct edge_tolled_t
{
	using kind = boost::edge_property_tag;
};
const edge_tolled_t edge_tolled;

struct problem
{
	using edge_property_type = boost::property<boost::edge_weight_t, cost_type,
		boost::property<edge_tolled_t, bool>>;

	using graph_type = boost::adjacency_list<boost::vecS,
		boost::vecS,
		boost::directedS,
		boost::no_property,
		edge_property_type>;

	graph_type graph;
	std::vector<commodity> commodities;

	static std::vector<problem> read_from_json(std::string filename);
	static void write_to_json(std::string filename, const std::vector<problem>& problems);

	void write_to_json(std::string filename) const;
};
