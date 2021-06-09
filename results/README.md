## Experimental Results
The file `results.txt` contains the results of the experiments. The file is tab-delimited and consists of 9 columns:

- Name of the model
- Test problem instance
- Time (in seconds)
- Objective value of the best solution
- Best (upper) bound
- Optimality gap (in percentage)
- Number of searched nodes
- Relaxation at root node
- The time spent for path enumeration

The names of the models follow the syntax `(model)-(breakpoint)` where:
- `(model) in [std, vf, apstd, pvf, cs1/2, vfcs1/2, apcs1/2, pcs1/2]` is the model code as in the paper, except for `apstd` and `apcs` which are coded as `(PASTD)` and `(PACS)` in the paper.
- `(breakpoint) in [10, 20, 50, 100, 200, 500, 1000]` is the breakpoint N in the paper. The path enumeration algorithm only produces at most N paths. So for commodities with more than N paths, a fallback algorithm is used instead (unprocessed `(STD)`).

The `(breakpoint)` for `std` is extended to `100000` to prove the robustness of the hybrid framework.

The test instances could be found in `probs/` subfolder. The name of each test instance follows the syntax `(class)(commodities)-(index)` where:

- `(class) in [g, h, d, v]` is the instance class as in the paper (grid, delaunay, or voronoi)
- `(commodities) in range(30, 50, 5)` is the number of commodities
- `(index) in range(1, 10)` is the index of the instance in the set
