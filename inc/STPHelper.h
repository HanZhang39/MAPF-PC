#pragma once

#include "stp/TemporalGraph.hpp"
#include "SingleAgentSolver.h"



class STPHelper {

private:
  // TP and agent_lb_ub that only considers the distance heuristic
  TemporalGraph tp;
  vector<vector<pair<int,int>>> agent_lb_ub;
  bool enabled = true;

  CBSNode* prev_parent= nullptr;
  distance_matrix_t dm_parent;

	const vector<ConstraintTable>& initial_constraints;
	const vector<SingleAgentSolver*>& search_engines;

public:

	double accumulated_runtime = 0;  // runtime of building MDDs

  STPHelper(const vector<ConstraintTable>& initial_constraints,
            const vector<SingleAgentSolver*>& search_engines):
    tp(),
    initial_constraints(initial_constraints), search_engines(search_engines) {}


  void set_flag(bool flag){ enabled = flag; };

  /*
    propagate_root

    generate tp and agent_lb_ub from the search_engines.
   */
  void propagate_root(CBSNode* root, vector<ConstraintTable>& initial_constraints);

  /*
    propagate

    Invoked when generating child.
    It copies the STN in parent, adds the new temporal constraints in child.
    Adds inferred temporal constraints to child;

    return True iff the STN does not have a negative circle
  */
  bool propagate(CBSNode* parent, CBSNode* child);

};
