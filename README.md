# A Catalog of Formulations for the Network Pricing Problem

This is the code repository for the paper: [A Catalog of Formulations for the Network
Pricing Problem](https://arxiv.org/pdf/2106.03887.pdf) by Quang Minh Bui, Bernard Gendron, and Margarida Carvalho from the Université de Montréal.

## Installation

### Requirements

The code is written in C++ and should be compiled on a Linux machine. These libraries and programs should be available:

- `cmake` (version 3.8)
- `make`
- A C++ compiler (such as `g++`)
- Boost's `program_options`
- CPLEX 12 (make sure the headers and the `.a` files are linked in `/usr/include` and `/usr/lib` folders)

### Compilation

First, create a folder and clone this repository into a subfolder named `src/`:

```
git clone (git link here) src
```

Then, create another subfolder named `build/` and open it:

```
mkdir build
cd build
```

Run `cmake` and `make`:

```
cmake ../src
make
```

If the compilation is successful, an executable program should be available in the `netpricing/` folder. Test it by running:

```
netpricing/netpricing --help
```

A help message should appear.

## Run the Experiments

The program contains a lot of old experiments and models that are not mentioned in the paper. We are interested only in those experiments described in the paper.

### Generation of Problem Instances

Running the program without any argument will create a random problem instance and write it to `report.json` (overwrite any existing one).

```
$ ./netpricing
RANDOM NETWORK created:
    n = 10
    a = 20
    k = 5
    p = 0.2
...
```

The parameters for the generator:

- `n`: The number of nodes
- `a`: The number of arcs
- `k`: The number of commodities
- `p`: The proportion of the tolled arcs

To change a (or multiple) parameter, append it as an argument:

```
$ ./netpricing -n 50 -a 200 -k 30 -p 0.1
RANDOM NETWORK created:
    n = 50
    a = 200
    k = 30
    p = 0.1
...
```

To redirect the output (instead of `report.json`), use the `-o` option (overwrite any existing file):

```
$ ./netpricing -o prob1.json
```

To reuse a generated problem as input, use the `-i` option (if there is no `-i` option, a random problem is created):

```
$ ./netpricing -i prob1.json
NETWORK imported from prob1.json
...
```

An output file (default `report.json`) always contains a copy of the generated or imported problem instance, so it can always be used as an input.

To generate a problem with specific topology, use `--grid (width) (height)`, `--delaunay (nodes)`, and `--voronoi (nodes)` (will ignore `-n` and `-a`):

```
$ ./netpricing --grid 5 12 -k 30 -o g30.json
GRID NETWORK created:
    width  = 5
    height = 12
    n      = 60
    a      = 206
    k      = 30
    p      = 0.2
...
$ ./netpricing --delaunay 144 -k 30 -o d30.json
DELAUNAY NETWORK created:
    n = 144
    a = 836
    k = 30
    p = 0.2
...
$ ./netpricing --voronoi 144 -k 30 -o v30.json
VORONOI NETWORK created:
    n         = 144
    a         = 410
    k         = 30
    p         = 0.2
...
```

### Running a Hybrid Model

A hybrid model is a model consisting of multiple formulations, separated by a breakpoint number `N`. Commodities with less than or equal `N` paths will use the first formulation (main), and commodities with more than `N` paths will use the second formulation (fallback). To run a hybrid model, use the `-c` option (`c` for `compose`):

```
$ ./netpricing -c std-100-ustd
$ ./netpricing -c vf-1000-spgm
$ ./netpricing -c pcs2-20
```

The syntax for a hybrid model is `(main)-N-(fallback)`. In the first command, `(main)=std` (standard formulation) and `(fallback)=ustd` (unprocessed standard). The breakpoint is `100`, which means commodities with at most 100 paths will use the `std` formulation, the rest will use `ustd` formulation. The fallback formulation can be omitted as in the third command, and the default fallback would be `ustd`.

There are 14 valid formulations for `(main)` and only 2 for `(fallback)`:

| Code    | Name in paper        | Main | Fallback |
| ------- | -------------------- | ---- | -------- |
| `std`   | (STD)                | ✓    |          |
| `vf`    | (VF)                 | ✓    |          |
| `apstd` | (PASTD)              | ✓    |          |
| `pvf`   | (PVF)                | ✓    |          |
| `cs1`   | (CS1)                | ✓    |          |
| `vfcs1` | (VFCS1)              | ✓    |          |
| `apcs1` | (PACS1)              | ✓    |          |
| `pcs1`  | (PCS1)               | ✓    |          |
| `cs2`   | (CS2)                | ✓    |          |
| `vfcs2` | (VFCS2)              | ✓    |          |
| `apcs2` | (PACS2)              | ✓    |          |
| `pcs2`  | (PCS2)               | ✓    |          |
| `ustd`  | Unprocessed (STD)    | ✓    | ✓        |
| `spgm`  | SPGM-processed (STD) | ✓    | ✓        |

One can also create hybrid models with multiple breakpoints `(main1)-N-(main2)-M-(fallback)`, so that commodities with `p <= N` paths will use `(main1)`, with `N < p <= M` paths will use `(main2)`, and with `p > M` paths will use `(fallback)`.

There are also several options that users should consider:

| Option         | Example   | Meaning                                |
| -------------- | --------- | -------------------------------------- |
| `-T (time)`    | `-T 3600` | Maximum time to run (in seconds)       |
| `-t (threads)` | `-t 1`    | Number of threads                      |
| `-R`           | `-R`      | Only solve the relaxation at root node |

### Comparison between Path-based and SPGM Preprocessings
Some experiments does not involve solving a problem using a model, for example, comparing the number of nodes reduced by preprocessing methods. These experiments are implemented as "routines", marked by the option `-r (index)`. If the `-r` option is present, the program breaks away from its normal operation and run a specific program specified by the `(index)`. Additional arguments for routines are provided with the `--args` option.

In the paper, routines are used for the experiment comparing the path-based and SPGM preprocessings. First, open the folder containing all problem instances (available in `results/probs`). Then, run the command:

```
$ netpricing -r 27 --args (class) 10000 > data.txt
```

where `(class) in [g, h, d, v]` is the prefix of the problem class. The program will enumerate the paths of all commodities of all instances with prefix `(class)` in the current folder. The maximum number of paths for each commodity is 10000. The results will be written in the `data.txt` file, where each row corresponds to a commodity. Each row has 7 entries:

- The number of paths (10001 if there are more than 10000 paths).
- The number of nodes, arc, tolled arcs after preprocessing (path-based). If there are more than 10000 paths, path-based preprocessing cannot apply and these entries will be `-1`.
- The number of nodes, arc, tolled arcs after preprocessing (SPGM).