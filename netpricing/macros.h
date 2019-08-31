#pragma once

#define LOOP(i, size) for (int i = 0; i < size; ++i)
#define A1_TO_A(prob, a) (prob).alledges_index_map.right.at((prob).tolled_index_map.left.at(a))
#define A2_TO_A(prob, a) (prob).alledges_index_map.right.at((prob).tollfree_index_map.left.at(a))
#define A_TO_A1(prob, a) (prob).tolled_index_map.right.at((prob).alledges_index_map.left.at(a))
#define A_TO_A2(prob, a) (prob).tollfree_index_map.right.at((prob).alledges_index_map.left.at(a))
