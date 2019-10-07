#pragma once

#define LOOP(i, size) for (int i = 0; i < size; ++i)
#define A1_TO_A(prob, a) (prob).alledges_index_map.right.at((prob).tolled_index_map.left.at(a))
#define A2_TO_A(prob, a) (prob).alledges_index_map.right.at((prob).tollfree_index_map.left.at(a))
#define A_TO_A1(prob, a) (prob).tolled_index_map.right.at((prob).alledges_index_map.left.at(a))
#define A_TO_A2(prob, a) (prob).tollfree_index_map.right.at((prob).alledges_index_map.left.at(a))

#define A_TO_EDGE(prob, a) (prob).alledges_index_map.left.at(a)
#define A1_TO_EDGE(prob, a) (prob).tolled_index_map.left.at(a)
#define A2_TO_EDGE(prob, a) (prob).tollfree_index_map.left.at(a)

#define SRC_DST(prob, edge) int src = source((edge), (prob).graph); int dst = target((edge), (prob).graph);
#define SRC_DST_FROM_A(prob, a) auto edge = A_TO_EDGE(prob, a); SRC_DST(prob, edge)
#define SRC_DST_FROM_A1(prob, a) auto edge = A1_TO_EDGE(prob, a); SRC_DST(prob, edge)
#define SRC_DST_FROM_A2(prob, a) auto edge = A2_TO_EDGE(prob, a); SRC_DST(prob, edge)