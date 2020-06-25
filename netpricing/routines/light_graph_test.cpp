#include "../graph/light_graph.h"
#include "../graph/cplex_graph.h"
#include "../problem_generator.h"

#include <chrono>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace std;
using namespace boost;

inline std::ostream& operator<<(std::ostream& os, const std::vector<std::pair<int, int>>& input)
{
	for (auto const& i : input) {
		os << i.first << "-" << i.second << " ";
	}
	return os;
}

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

void light_graph_bilevel_feasible_yen_acctest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible (Yen's version) accuracy test..." << endl;

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

		vector<path> paths = lgraph.bilevel_feasible_paths_yen(from, to, 1000);

		// Verify that the toll sets are bilevel feasible
		using iipair = pair<int, int>;
		vector<set<iipair>> all_sets;

		for (const path& path : paths) {
			// Get the set of toll arcs
			set<iipair> tset = lgraph.get_toll_set(path);

			// Check if it is not a superset of any previous set
			if (std::any_of(all_sets.begin(), all_sets.end(),
							[&](const auto& s) {
								return std::includes(tset.begin(), tset.end(), s.begin(), s.end());
							})) {
				cerr << "Toll set is not bilevel feasible" << endl;
				return;
			}
			else {
				all_sets.emplace_back(std::move(tset));
			}
		}
	}

	cout << "Light graph Bilevel feasible (Yen's version) produced accurate results" << endl;
}

void light_graph_bilevel_feasible_yen_perftest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible (Yen's version) (1000 paths, 5 x 12 grid) performance test..." << endl;

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

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_yen(from, to, 1000);
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Bilevel feasible: " << time * 1000 / TOTAL << " ms" << endl;
}

void light_graph_bilevel_feasible_acctest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible accuracy test..." << endl;

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

		vector<path> paths_yen = lgraph.bilevel_feasible_paths_yen(from, to, 1000);
		vector<path> paths_new = lgraph.bilevel_feasible_paths(from, to, paths_yen.size());

		// Verify that the costs are in ascending order
		cost_type last_cost = lgraph.get_path_cost(paths_new[0]);
		for (int j = 1; j < paths_new.size(); j++) {
			cost_type cost = lgraph.get_path_cost(paths_new[j]);
			if (cost - last_cost < -1e-3) {
				cerr << "Costs not in ascending order (" << cost << " < " << last_cost << ")" << endl;
				break;
			}
			last_cost = cost;
		}

		// Verify that the toll sets are bilevel feasible
		using iipair = pair<int, int>;
		vector<set<iipair>> all_sets;
		for (const path& path : paths_new) {
			// Get the set of toll arcs
			auto tset = lgraph.get_toll_set(path);

			// Check if it is not a superset of any previous set
			if (std::any_of(all_sets.begin(), all_sets.end(),
							[&](const auto& s) {
								return std::includes(tset.begin(), tset.end(), s.begin(), s.end());
							})) {
				cerr << "Toll set is not bilevel feasible" << endl;
				break;
			}
			else {
				all_sets.emplace_back(std::move(tset));
			}
		}

		// Verify the two lists are the same
		if (paths_yen != paths_new) {
			cerr << "Yen version:" << endl;
			for (const path& p : paths_yen) {
				auto tlist = lgraph.get_toll_list(p);
				cost_type cost = lgraph.get_path_cost(p);
				cerr << "  " << tlist << "    " << cost << endl;
			}
			cerr << "New version:" << endl;
			for (const path& p : paths_new) {
				auto tlist = lgraph.get_toll_list(p);
				cost_type cost = lgraph.get_path_cost(p);
				cerr << "  " << tlist << "    " << cost << endl;
			}
			cerr << "List of bilevel feasible paths are not the same" << endl;
			return;
		}
	}

	cout << "Light graph Bilevel feasible produced accurate results" << endl;
}

void light_graph_bilevel_feasible_perftest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible (10000 paths, 5 x 12 grid) performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<light_graph> lgraphs;
	vector<int> froms, tos;
	vector<int> sizes(SAMPLES);

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
			vector<path> ps = lgraph.bilevel_feasible_paths_yen(from, to, 10000);
			if (j == 0)
				sizes[i] = ps.size();
		}
	}

	auto end = chrono::high_resolution_clock::now();
	auto time = chrono::duration<double>(end - start).count();

	cout << "Yen version: " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths(from, to, sizes[i]);
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "New version: " << time * 1000 / TOTAL << " ms" << endl;

	cout << "Average num paths: " << std::accumulate(sizes.begin(), sizes.end(), 0) / SAMPLES << endl;
}

void light_graph_bilevel_feasible_stat() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible (5 x 12 grid, K = 40) statistics..." << endl;

	const int SAMPLES = 100;
	const int MAX_PATHS = 500;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<problem> probs;

	LOOP(i, SAMPLES) {
		probs.emplace_back(random_grid_problem(5, 12, 40, 0.2, random_engine));
	}

	LOOP(i, SAMPLES) {
		problem& prob = probs[i];
		light_graph lgraph(prob.graph);

		LOOP(k, prob.commodities.size()) {
			commodity& comm = prob.commodities[k];
			vector<path> ps = lgraph.bilevel_feasible_paths(comm.origin, comm.destination, MAX_PATHS);
			if (ps.size() < MAX_PATHS || lgraph.is_toll_free(ps.back()))
				cout << ps.size() << endl;
			else
				cout << MAX_PATHS + 1 << endl;
		}
	}
}

void light_graph_bilevel_feasible_2_acctest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible v2 accuracy test..." << endl;

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

		vector<path> paths_v1 = lgraph.bilevel_feasible_paths(from, to, 500);
		vector<path> paths_v2 = lgraph.bilevel_feasible_paths_2(from, to, 500);

		// Verify that the costs are in ascending order
		cost_type last_cost = lgraph.get_path_cost(paths_v2[0]);
		for (int j = 1; j < paths_v2.size(); j++) {
			cost_type cost = lgraph.get_path_cost(paths_v2[j]);
			if (cost - last_cost < -1e-3) {
				cerr << "Costs not in ascending order (" << cost << " < " << last_cost << ")" << endl;
				break;
			}
			last_cost = cost;
		}

		// Verify that the toll sets are bilevel feasible
		using iipair = pair<int, int>;
		vector<set<iipair>> all_sets;
		for (const path& path : paths_v2) {
			// Get the set of toll arcs
			auto tset = lgraph.get_toll_set(path);

			// Check if it is not a superset of any previous set
			if (std::any_of(all_sets.begin(), all_sets.end(),
							[&](const auto& s) {
								return std::includes(tset.begin(), tset.end(), s.begin(), s.end());
							})) {
				cerr << "Toll set is not bilevel feasible" << endl;
				break;
			}
			else {
				all_sets.emplace_back(std::move(tset));
			}
		}

		// Verify the two lists are the same
		if (paths_v1 != paths_v2) {
			cerr << "Version 1:" << endl;
			for (const path& p : paths_v1) {
				auto tlist = lgraph.get_toll_list(p);
				cost_type cost = lgraph.get_path_cost(p);
				cerr << "  " << tlist << "    " << cost << endl;
			}
			cerr << "Version 2:" << endl;
			for (const path& p : paths_v2) {
				auto tlist = lgraph.get_toll_list(p);
				cost_type cost = lgraph.get_path_cost(p);
				cerr << "  " << tlist << "    " << cost << endl;
			}
			cerr << "List of bilevel feasible paths are not the same" << endl;
			return;
		}
	}

	cout << "Light graph Bilevel feasible v2 produced accurate results" << endl;
}

void light_graph_bilevel_feasible_2_perftest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible v2 (12 x 12 grid) performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<light_graph> lgraphs;
	vector<int> froms, tos;
	vector<int> sizes1(SAMPLES);
	vector<int> sizes2(SAMPLES);
	vector<int> sizes2u(SAMPLES);

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(12, 12, 1, 0.2, random_engine);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths(from, to, 200);
			if (j == 0)
				sizes1[i] = ps.size();
		}
	}

	auto end = chrono::high_resolution_clock::now();
	auto time = chrono::duration<double>(end - start).count();

	cout << "Version 1 (max 200): " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_2(from, to, 1000);
			if (j == 0)
				sizes2[i] = ps.size();
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Version 2 (max 1000): " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_2(from, to, 1000, false);
			if (j == 0)
				sizes2u[i] = ps.size();
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Version 2u (max 1000): " << time * 1000 / TOTAL << " ms" << endl;

	cout << "Average num paths 1: " << std::accumulate(sizes1.begin(), sizes1.end(), 0) / SAMPLES << endl;
	cout << "Average num paths 2: " << std::accumulate(sizes2.begin(), sizes2.end(), 0) / SAMPLES << endl;
	cout << "Average num paths 2u: " << std::accumulate(sizes2u.begin(), sizes2u.end(), 0) / SAMPLES << endl;
}

void light_graph_bilevel_feasible_3_acctest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible v3 accuracy test..." << endl;

	const int SAMPLES = 100;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<cplex_graph> lgraphs;
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

		vector<path> paths_v2 = lgraph.bilevel_feasible_paths_2(from, to, 500);
		vector<path> paths_v3 = lgraph.bilevel_feasible_paths_3(from, to, 500);

		// Verify that the costs are in ascending order
		cost_type last_cost = lgraph.get_path_cost(paths_v3[0]);
		for (int j = 1; j < paths_v3.size(); j++) {
			cost_type cost = lgraph.get_path_cost(paths_v3[j]);
			if (cost - last_cost < -1e-3) {
				cerr << "Costs not in ascending order (" << cost << " < " << last_cost << ")" << endl;
				break;
			}
			last_cost = cost;
		}

		// Verify that the toll sets are bilevel feasible
		using iipair = pair<int, int>;
		vector<set<iipair>> all_sets;
		for (const path& path : paths_v3) {
			// Get the set of toll arcs
			auto tset = lgraph.get_toll_set(path);

			// Check if it is not a superset of any previous set
			if (std::any_of(all_sets.begin(), all_sets.end(),
							[&](const auto& s) {
								return std::includes(tset.begin(), tset.end(), s.begin(), s.end());
							})) {
				cerr << "Toll set is not bilevel feasible" << endl;
				break;
			}
			else {
				all_sets.emplace_back(std::move(tset));
			}
		}

		// Verify the two lists are the same
		if (paths_v2 != paths_v3) {
			cerr << "Version 2:" << endl;
			for (const path& p : paths_v2) {
				auto tlist = lgraph.get_toll_list(p);
				cost_type cost = lgraph.get_path_cost(p);
				cerr << "  " << tlist << "    " << cost << endl;
			}
			cerr << "Version 3:" << endl;
			for (const path& p : paths_v3) {
				auto tlist = lgraph.get_toll_list(p);
				cost_type cost = lgraph.get_path_cost(p);
				cerr << "  " << tlist << "    " << cost << endl;
			}
			cerr << "List of bilevel feasible paths are not the same" << endl;
			return;
		}
	}

	cout << "Light graph Bilevel feasible v3 produced accurate results" << endl;
}

void light_graph_bilevel_feasible_3_perftest() {
	using path = vector<int>;

	cout << "Light graph Bilevel feasible v3 (12 x 12 grid) performance test..." << endl;

	const int SAMPLES = 100;
	const int REPEAT = 1;
	const int TOTAL = SAMPLES * REPEAT;

	auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
	auto random_engine = default_random_engine(seed);

	vector<cplex_graph> lgraphs;
	vector<int> froms, tos;
	vector<int> sizes2(SAMPLES);
	vector<int> sizes2u(SAMPLES);
	vector<int> sizes3(SAMPLES);
	vector<int> sizes3u(SAMPLES);

	LOOP(i, SAMPLES) {
		problem prob = random_grid_problem(12, 12, 1, 0.2, random_engine);
		lgraphs.emplace_back(prob.graph);
		froms.push_back(prob.commodities[0].origin);
		tos.push_back(prob.commodities[0].destination);
	}

	auto start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_2(from, to, 1000);
			if (j == 0)
				sizes2[i] = ps.size();
		}
	}

	auto end = chrono::high_resolution_clock::now();
	auto time = chrono::duration<double>(end - start).count();

	cout << "Version 2 (max 1000): " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_2(from, to, 1000, false);
			if (j == 0)
				sizes2u[i] = ps.size();
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Version 2u (max 1000): " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_3(from, to, 1000);
			if (j == 0)
				sizes3[i] = ps.size();
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Version 3 (max 1000): " << time * 1000 / TOTAL << " ms" << endl;

	start = chrono::high_resolution_clock::now();

	LOOP(i, SAMPLES) {
		int from = froms[i], to = tos[i];
		auto& lgraph = lgraphs[i];

		LOOP(j, REPEAT) {
			vector<path> ps = lgraph.bilevel_feasible_paths_3(from, to, 1000, false);
			if (j == 0)
				sizes3u[i] = ps.size();
		}
	}

	end = chrono::high_resolution_clock::now();
	time = chrono::duration<double>(end - start).count();

	cout << "Version 3u (max 1000): " << time * 1000 / TOTAL << " ms" << endl;

	cout << "Average num paths 2: " << std::accumulate(sizes2.begin(), sizes2.end(), 0) / SAMPLES << endl;
	cout << "Average num paths 2u: " << std::accumulate(sizes2u.begin(), sizes2u.end(), 0) / SAMPLES << endl;
	cout << "Average num paths 3: " << std::accumulate(sizes3.begin(), sizes3.end(), 0) / SAMPLES << endl;
	cout << "Average num paths 3u: " << std::accumulate(sizes3u.begin(), sizes3u.end(), 0) / SAMPLES << endl;
}
