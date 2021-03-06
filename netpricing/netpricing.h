// netpricing.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>

#include <boost/graph/graph_traits.hpp>
#include <ilcplex/ilocplex.h>

#include "problem.h"
#include "problem_multi.h"
#include "problem_generator.h"

#include "models/model_cplex.h"
#include "models/standard_model.h"
#include "models/standard_goal_model.h"
#include "models/standard_cscut_model.h"
#include "models/standard_vfcut_model.h"
#include "models/light_vfcut_model.h"
#include "models/benders_model_original.h"
#include "models/benders_model_reduced.h"
#include "models/benders_model_reduced2.h"
#include "models/value_func_model.h"
#include "models/benders_xt_model.h"
#include "models/benders_xy_model.h"
#include "models/benders_xyt_model.h"
#include "models/slackbranch_model.h"
#include "models/compslack_model.h"

#include "models/standard_model_multi.h"

#include "hybrid/standard_hmodel.h"
#include "hybrid/value_func_hmodel.h"
#include "hybrid/path_hmodel.h"
#include "hybrid/vfpath_hmodel.h"
#include "hybrid/path_didi_hmodel.h"
#include "hybrid/filtered_hmodel.h"
#include "hybrid/processed_hmodel.h"
#include "hybrid/spgm_hmodel.h"
#include "hybrid/processed_spgm_hmodel.h"
#include "hybrid/arc_path_hmodel.h"
#include "hybrid/composed_hmodel.h"

#include "csenum/csenum.h"
#include "csenum/csenum_excl.h"

#include "routines/routines.h"

#include "macros.h"
