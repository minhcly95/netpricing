#include "../graph/light_graph.h"
#include "../problem_generator.h"

#include <chrono>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;

void light_graph_dijkstra_acctest() {
	using path = vector<int>;

	cout << "Light graph Dijkstra accuracy test..." << endl;

	const int SAMPLES = 1000;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<problem::graph_type> graphs;
	vector<light_graph> lgraphs;
	vector<int> froms, tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		graphs.emplace_back(prob.graph);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];
		auto& graph = graphs[i];

		// Light graph version
		path path1 = lgraph.shortest_path(from, to);

		// Boost version
		vector<int> parents(num_vertices(graph));
		auto index_map = boost::get(vertex_index, graph);
		auto parent_map = make_iterator_property_map(parents.begin(), index_map);

		dijkstra_shortest_paths(graph, from, predecessor_map(parent_map));

		// Trace back the path
		path path2;
		int curr = to;
		while (curr != from) {
			path2.push_back(curr);
			curr = parents[curr];
		}
		path2.push_back(from);
		std::reverse(path2.begin(), path2.end());

		cost_type cost1, cost2;
		cost1 = lgraph.get_path_cost(path1);
		cost2 = lgraph.get_path_cost(path2);

		if (abs(cost1 - cost2) > 0.01) {
			cerr << "Test failed (" << cost1 << " != " << cost2 << ")" << endl;
			return;
		}
	}

	cout << "Light graph Dijkstra produced accurate results" << endl;
}

void light_graph_dijkstra_perftest() {
	using path = vector<int>;

	cout << "Light graph Dijkstra (5 x 12 grid) performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 100;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<problem::graph_type> graphs;
	vector<light_graph> lgraphs;
	vector<int> froms, tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		graphs.emplace_back(prob.graph);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	auto start1 = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			path p = lgraph.shortest_path(from, to);
		}
	}

	auto end1 = chrono::high_resolution_clock::now();
	double time1 = chrono::duration<double>(end1 - start1).count();

	auto start2 = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& graph = graphs[i];

		LOOP(j, REPEAT) {
			vector<int> parents(num_vertices(graph));
			auto index_map = boost::get(vertex_index, graph);
			auto parent_map = make_iterator_property_map(parents.begin(), index_map);

			dijkstra_shortest_paths(graph, from, predecessor_map(parent_map));

			// Trace back the path
			path path;
			int curr = to;
			while (curr != from) {
				path.push_back(curr);
				curr = parents[curr];
			}
			path.push_back(from);
			std::reverse(path.begin(), path.end());
		}
	}

	auto end2 = chrono::high_resolution_clock::now();
	double time2 = chrono::duration<double>(end2 - start2).count();

	cout << "Light graph: " << time1 * 1000 / TOTAL << " ms" << endl;
	cout << "Boost graph: " << time2 * 1000 / TOTAL << " ms" << endl;
}

void light_graph_yen_acctest() {
	using path = vector<int>;

	cout << "Light graph Yen accuracy test..." << endl;

	const int SAMPLES = 1000;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<light_graph> lgraphs;
	vector<int> froms, tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		vector<path> paths = lgraph.k_shortest_paths(from, to, 100);

		// Verify that the costs are in ascending order
		cost_type last_cost = lgraph.get_path_cost(paths[0]);
		for (int j = 1; j < paths.size(); j++) {
			cost_type cost = lgraph.get_path_cost(paths[j]);
			if (cost - last_cost < -1e-3) {
				cerr << "Costs not in ascending order (" << cost << " < " << last_cost << ")" << endl;
				return;
			}
			last_cost = cost;
		}

		// Test for duplicated nodes
		for (path p : paths) {
			std::sort(p.begin(), p.end());
			if (std::unique(p.begin(), p.end()) != p.end()) {
				cerr << "Duplicated node" << endl;
				return;
			}
		}

		// Test for duplicated paths (destroy order of paths)
		std::sort(paths.begin(), paths.end());
		if (std::unique(paths.begin(), paths.end()) != paths.end()) {
			cerr << "Duplicated path" << endl;
			return;
		}
	}

	cout << "Light graph Yen produced accurate results" << endl;
}

void light_graph_yen_perftest() {
	using path = vector<int>;

	cout << "Light graph Yen (1000 paths, 5 x 12 grid) performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<light_graph> lgraphs;
	vector<int> froms, tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.k_shortest_paths(from, to, 1000);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "Average time: " << time * 1000 / TOTAL << " ms" << endl;
}

void light_graph_toll_unique_acctest() {
	using path = vector<int>;

	cout << "Light graph Toll unique accuracy test..." << endl;

	const int SAMPLES = 100;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<light_graph> lgraphs;
	vector<int> froms, tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		vector<path> paths = lgraph.toll_unique_paths(from, to, 1000);

		// Verify that the toll sets are unique
		using odpair = pair<int, int>;
		set<set<odpair>> all_sets;

		for (const path& path : paths) {
			// Get the set of toll arcs
			set<odpair> toll_set;
			for (int i = 0; i < path.size() - 1; i++)
				if (lgraph.edge(path[i], path[i + 1]).is_tolled)
					toll_set.emplace(path[i], path[i + 1]);

			// Add to all sets, throw if existed
			if (!all_sets.insert(toll_set).second) {
				cerr << "Toll set is not unique" << endl;
				return;
			}
		}
	}

	cout << "Light graph Toll unique produced accurate results" << endl;
}

void light_graph_toll_unique_perftest() {
	using path = vector<int>;

	cout << "Light graph Toll unique (1000 paths, 5 x 12 grid) performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<light_graph> lgraphs;
	vector<int> froms, tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.k_shortest_paths(from, to, 1000);
		}
	}

	auto end = chrono::high_resolution_clock::now();
	double time = chrono::duration<double>(end - start).count();

	cout << "K paths: " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.toll_unique_paths(from, to, 1000);
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Toll unique: " << time * 1000 / TOTAL << " ms" << endl;
}

void light_graph_price_from_src_acctest() {
	using path = vector<int>;

	cout << "Light graph Price from src accuracy test..." << endl;

	const int SAMPLES = 1000;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<problem::graph_type> graphs;
	vector<light_graph> lgraphs;
	vector<int> froms;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		graphs.emplace_back(prob.graph);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
	}

	LOOP(i, SAMPLES) {
		int from = froms[i];
		auto& lgraph = lgraphs[i];
		auto& graph = graphs[i];

		vector<cost_type> prices = lgraph.price_from_src(from);

		LOOP(v, lgraph.V) {
			path p = lgraph.shortest_path(from, v);
			cost_type cost = lgraph.get_path_cost(p);

			if (abs(cost - prices[v]) > 0.01) {
				cerr << "Test failed (" << cost << " != " << prices[v] << ")" << endl;
				return;
			}
		}
	}

	cout << "Light graph Price from src produced accurate results" << endl;
}

void light_graph_price_to_dst_acctest() {
	using path = vector<int>;

	cout << "Light graph Price to dst accuracy test..." << endl;

	const int SAMPLES = 1000;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<problem::graph_type> graphs;
	vector<light_graph> lgraphs;
	vector<int> tos;

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(5, 12, 1, 0.2, random_engine);
		graphs.emplace_back(prob.graph);
		lgraphs.emplace_back(prob.graph);
		tos.push_back(prob.commodities[0].destination);
	}

	LOOP(i, SAMPLES) {
		int to = tos[i];
		auto& lgraph = lgraphs[i];
		auto& graph = graphs[i];

		vector<cost_type> prices = lgraph.price_to_dst(to);

		LOOP(v, lgraph.V) {
			path p = lgraph.shortest_path(v, to);
			cost_type cost = lgraph.get_path_cost(p);

			if (abs(cost - prices[v]) > 0.01) {
				cerr << "Test failed (" << cost << " != " << prices[v] << ")" << endl;
				return;
			}
		}
	}

	cout << "Light graph Price to dst produced accurate results" << endl;
}