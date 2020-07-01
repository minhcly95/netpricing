#pragma once

#include <string>

void follower_solver_perftest();
void follower_cplex_solver_perftest();
void follower_light_solver_perftest();
void follower_solver_acctest();

void inverse_solver_acctest();
void inverse_solver_perftest();

void tolls_heuristic_perftest();

void light_graph_dijkstra_acctest();
void light_graph_dijkstra_perftest();
void light_graph_yen_acctest();
void light_graph_yen_perftest();
void light_graph_toll_unique_acctest();
void light_graph_toll_unique_perftest();
void light_graph_price_from_src_acctest();
void light_graph_price_to_dst_acctest();
void light_graph_bilevel_feasible_yen_acctest();
void light_graph_bilevel_feasible_yen_perftest();
void light_graph_bilevel_feasible_acctest();
void light_graph_bilevel_feasible_perftest();
void light_graph_bilevel_feasible_stat();
void light_graph_bilevel_feasible_2_acctest();
void light_graph_bilevel_feasible_2_perftest();
void light_graph_bilevel_feasible_3_acctest();
void light_graph_bilevel_feasible_3_perftest();

void path_vs_spgm_preprocessors_compare();

void data_numpaths_stats(std::string prefix);
