#pragma once

#include <algorithm>
#include <vector>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/strong_components.hpp>

struct reachability_visitor : public boost::dfs_visitor<> {
	reachability_visitor(int origin, int destination, bool& is_reachable) :
		_origin(origin), _destination(destination), _is_reachable(is_reachable) {}

	// If we start on another vertex, then we have scanned all reachable vertices from origin
	template <class vertex_type, class graph_type>
	void start_vertex(vertex_type u, const graph_type& g) {
		if (u != _origin) {
			_is_reachable = false;
			throw _is_reachable;
		}
	}

	// Stop right after destination is discovered
	template <class vertex_type, class graph_type>
	void discover_vertex(vertex_type u, const graph_type& g) {
		if (u == _destination) {
			_is_reachable = true;
			throw _is_reachable;
		}
	}

protected:
	int _origin;
	int _destination;
	bool& _is_reachable;
};

template<class graph_type>
bool is_reachable(int origin, int destination, graph_type& graph) {
	using namespace std;
	using namespace boost;

	bool result = false;
	try {
		depth_first_search(graph, visitor(reachability_visitor(origin, destination, result)).root_vertex(origin));
	}
	catch (bool _) {}

	return result;
}

template<class graph_type>
bool is_strongly_connected(graph_type& graph) {
	using namespace std;
	using namespace boost;

	vector<int> component_map(num_vertices(graph));

	int num_components = strong_components(graph, make_iterator_property_map(component_map.begin(), get(vertex_index, graph)));

	return num_components == 1;
}