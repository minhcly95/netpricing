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

struct cycle_detection_visitor : public boost::dfs_visitor<> {
	cycle_detection_visitor(std::vector<int>& cycle) :
		_cycle(cycle), stack() {}

	// We build a path from the root of the tree
	template <class node_type, class graph_type>
	void discover_vertex(node_type u, const graph_type& g) {
		stack.push_back(u);
	}

	template <class node_type, class graph_type>
	void finish_vertex(node_type u, const graph_type& g) {
		stack.pop_back();
	}

	// If we encounter a back edge, then there is a cycle
	template <class edge_type, class graph_type>
	void back_edge(edge_type e, const graph_type& g) {
		int v = boost::target(e, g);
		auto pos_v = std::find(stack.begin(), stack.end(), v);
		std::copy(pos_v, stack.end(), std::back_inserter(_cycle));
		throw true;
	}

protected:
	std::vector<int>& _cycle;
	std::vector<int> stack;
};

template<class graph_type>
std::vector<int> get_cycle(graph_type& graph) {
	using namespace std;
	using namespace boost;

	std::vector<int> result;
	try {
		depth_first_search(graph, visitor(cycle_detection_visitor(result)));
	}
	catch (bool _) {}

	return result;
}

template<class graph_type>
struct tree_path_visitor : public boost::dfs_visitor<> {
	using vertex_type = typename boost::graph_traits<graph_type>::vertex_descriptor;
	using edge_type = typename boost::graph_traits<graph_type>::edge_descriptor;

	tree_path_visitor(int origin, int destination, std::vector<edge_type>& path) :
		_origin(origin), _destination(destination), _path(path) {}

	// If we start on another vertex, then we have scanned all reachable vertices from origin
	void start_vertex(vertex_type u, const graph_type& g) {
		if (u != _origin) {
			throw false;
		}
	}

	// We build a path from the root of the tree
	void examine_edge(edge_type e, const graph_type& g) {
		_path.push_back(e);
		if (boost::target(e, g) == _destination) {
			throw true;
		}
	}

	void finish_edge(edge_type e, const graph_type& g) {
		_path.pop_back();
	}

protected:
	int _origin;
	int _destination;
	std::vector<edge_type>& _path;
};

template<class graph_type>
std::vector<typename boost::graph_traits<graph_type>::edge_descriptor> get_tree_path(int origin, int destination, graph_type& graph) {
	using namespace std;
	using namespace boost;

	std::vector<typename boost::graph_traits<graph_type>::edge_descriptor> result;
	try {
		depth_first_search(graph, visitor(tree_path_visitor<graph_type>(origin, destination, result)).root_vertex(origin));
	}
	catch (bool _) {}

	return result;
}
