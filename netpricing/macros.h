#pragma once

#define LOOP(i, size) for (int i = 0; i < size; ++i)
#define A1_TO_A(prob, a) (prob).alledges_index_map.right.at((prob).tolled_index_map.left.at(a))
#define A2_TO_A(prob, a) (prob).alledges_index_map.right.at((prob).tollfree_index_map.left.at(a))
#define A_TO_A1(prob, a) (prob).tolled_index_map.right.at((prob).alledges_index_map.left.at(a))
#define A_TO_A2(prob, a) (prob).tollfree_index_map.right.at((prob).alledges_index_map.left.at(a))

#define A_TO_EDGE(prob, a) (prob).alledges_index_map.left.at(a)
#define A1_TO_EDGE(prob, a) (prob).tolled_index_map.left.at(a)
#define A2_TO_EDGE(prob, a) (prob).tollfree_index_map.left.at(a)

#define EDGE_FROM_SRC_DST(prob, src, dst) boost::edge(src, dst, (prob).graph).first
#define EDGE_TO_A(prob, e) (prob).alledges_index_map.right.at(e)
#define EDGE_TO_A1(prob, e) (prob).tolled_index_map.right.at(e)
#define EDGE_TO_A2(prob, e) (prob).tollfree_index_map.right.at(e)

#define SRC_DST(prob, edge) int src = source((edge), (prob).graph); int dst = target((edge), (prob).graph);
#define SRC_DST_FROM_A(prob, a) auto edge = A_TO_EDGE(prob, a); SRC_DST(prob, edge)
#define SRC_DST_FROM_A1(prob, a) auto edge = A1_TO_EDGE(prob, a); SRC_DST(prob, edge)
#define SRC_DST_FROM_A2(prob, a) auto edge = A2_TO_EDGE(prob, a); SRC_DST(prob, edge)

// Problem multi
#define A1_EXISTS(prob, k, a) ((prob).tolled_index_maps[k].left.find(a) != (prob).tolled_index_maps[k].left.end())
#define A_EXISTS(prob, k, a) ((prob).alledges_index_maps[k].left.find(a) != (prob).alledges_index_maps[k].left.end())
#define IF_A1_EXISTS(prob, k, a) if (A1_EXISTS(prob, k, a))
#define IF_A_EXISTS(prob, k, a) if (A_EXISTS(prob, k, a))

#define A1_TO_A_MULTI(prob, k, a) (prob).alledges_index_maps[k].right.at((prob).tolled_index_maps[k].left.at(a))
#define A2_TO_A_MULTI(prob, k, a) (prob).alledges_index_maps[k].right.at((prob).tollfree_index_maps[k].left.at(a))
#define A_TO_A1_MULTI(prob, k, a) (prob).tolled_index_maps[k].right.at((prob).alledges_index_maps[k].left.at(a))
#define A_TO_A2_MULTI(prob, k, a) (prob).tollfree_index_maps[k].right.at((prob).alledges_index_maps[k].left.at(a))