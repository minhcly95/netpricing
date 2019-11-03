#pragma once

#include <utility>
#include <vector>

#include <boost/bimap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <nlohmann/json.hpp>

#include "commodity.h"
#include "edge_tollfree_predicate.h"
#include "typedef.h"

struct edge_tolled_t
{
	using kind = boost::edge_property_tag;
};
const edge_tolled_t edge_tolled;

struct problem_base
{
	using edge_property_type = boost::property<boost::edge_weight_t, cost_type,
		boost::property<edge_tolled_t, bool>>;

	using graph_type = boost::adjacency_list<boost::vecS,
		boost::vecS,
		boost::directedS,
		boost::no_property,
		edge_property_type>;

	using edge_tolled_map_type = boost::property_map<graph_type, edge_tolled_t>::type;
	using edge_cost_map_type = boost::property_map<graph_type, boost::edge_weight_t>::type;

	using edge_iterator = boost::graph_traits<graph_type>::edge_iterator;
	using edge_descriptor = boost::graph_traits<graph_type>::edge_descriptor;
	using edge_index_bimap_type = boost::bimap<int, edge_descriptor>;

	using tollfree_graph_type = boost::filtered_graph<graph_type, edge_tollfree_predicate<edge_tolled_map_type>>;

	using cost_array = std::vector<cost_type>;
	using cost_matrix = std::vector<cost_array>;
	using big_m_type = cost_matrix;
	using big_n_type = cost_array;
};
