#pragma once

#include "CBSHeuristic.h"
#include "STPHelper.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "MutexReasoning.h"

class CBS
{
public:
  bool randomRoot = false; // randomize the order of the agents in the root CT node

  /////////////////////////////////////////////////////////////////////////////////////
  // stats
  double runtime = 0;
  double runtime_generate_child = 0; // runtime of generating child nodes
  double runtime_build_CT = 0; // runtime of building constraint table
  double runtime_build_CAT = 0; // runtime of building conflict avoidance table
  double runtime_path_finding = 0; // runtime of finding paths for single agents
  double runtime_detect_conflicts = 0;
  double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

  uint64_t num_corridor_conflicts = 0;
  uint64_t num_rectangle_conflicts = 0;
  uint64_t num_target_conflicts = 0;
  uint64_t num_mutex_conflicts = 0;
  uint64_t num_standard_conflicts = 0;

  uint64_t num_adopt_bypass = 0; // number of times when adopting bypasses

  uint64_t num_HL_expanded = 0;
  uint64_t num_HL_generated = 0;
  uint64_t num_LL_expanded = 0;
  uint64_t num_LL_generated = 0;


  CBSNode* dummy_start = nullptr;
  CBSNode* goal_node = nullptr;


  bool solution_found = false;
  int solution_cost = -2;
  double min_f_val;
  double focal_list_threshold;

  /////////////////////////////////////////////////////////////////////////////////////////
  // set params
  void setPrioritizeConflicts(bool p) { PC = p; heuristic_helper->PC = p; }
  void setRectangleReasoning(bool r) { rectangle_helper.use_rectangle_reasoning = r; heuristic_helper->rectangle_reasoning = r; }
  void setCorridorReasoning(bool c) { corridor_helper.use_corridor_reasoning = c; heuristic_helper->corridor_reasoning = c; }
  void setTargetReasoning(bool t) { target_reasoning = t; heuristic_helper->target_reasoning = t; }
  void setMutexReasoning(mutex_strategy m) {mutex_helper.strategy = m; heuristic_helper->mutex_reasoning = m; }
  void setDisjointSplitting(bool d) { disjoint_splitting = d; heuristic_helper->disjoint_splitting = d; }
  void setBypass(bool b) { bypass = b; } // 2-agent solver for heuristic calculation does not need bypass strategy.
  void setConflictSelectionRule(conflict_selection c) { conflict_selection_rule = c; heuristic_helper->conflict_seletion_rule = c; }
  void setNodeSelectionRule(node_selection n) { node_selection_rule = n; heuristic_helper->node_selection_rule = n; }
  void setNodeLimit(int n) { node_limit = n; }
  void setSTP(bool s) {stp_helper.set_flag(s); }
  void setUsingTimestamps(bool b) {
    for (auto ptr: search_engines){
      ptr->use_timestamps = b;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  // Runs the algorithm until the problem is solved or time is exhausted
  bool solve(double time_limit, int cost_lowerbound = 0, int cost_upperbound = MAX_COST);

  CBS(const Instance& instance, bool sipp, heuristics_type heuristic, int screen);
  CBS(vector<SingleAgentSolver*>& search_engines,
    const vector<ConstraintTable>& constraints,
      vector<Path>& paths_found_initially, heuristics_type heuristic, int screen);
  void clearSearchEngines();
  virtual  ~CBS();

  // Save results
  void saveResults(const string& fileName, const string& instanceName) const;

  void clear(); // used for rapid random  restart

  vector<ConstraintTable> initial_constraints;

protected:
  bool target_reasoning; // using target reasoning
  bool disjoint_splitting; // disjoint splitting
  bool mutex_reasoning; // using mutex reasoning
  bool bypass; // using Bypass1
  bool PC; // prioritize conflicts
  conflict_selection conflict_selection_rule;
  node_selection node_selection_rule;

  MDDTable mdd_helper;
  STPHelper stp_helper;
  RectangleReasoning rectangle_helper;
  CorridorReasoning corridor_helper;
  MutexReasoning mutex_helper;
  CBSHeuristic* heuristic_helper;

  pairing_heap<CBSNode*, compare<CBSNode::compare_node>> open_list;
  pairing_heap<CBSNode*, compare<CBSNode::secondary_compare_node>> focal_list;
  list<CBSNode*> allNodes_table;


  virtual string getSolverName() const;

  int screen;

  double time_limit;
  int node_limit = MAX_NODES;
  double focal_w = 1.0;
  int cost_upperbound = MAX_COST;


  clock_t start;

  int num_of_agents;


  vector<Path*> paths;
  vector<Path> paths_found_initially;  // contain initial paths found
  // vector<MDD*> mdds_initially;  // contain initial paths found
  vector<SingleAgentSolver*> search_engines;  // used to find (single) agents' paths and mdd

  // init helper
  void init_heuristic(heuristics_type heuristic);

  // high level search
  bool findPathForSingleAgent(CBSNode* node, int ag, int lower_bound = 0);
  virtual bool generateChild(CBSNode* child, CBSNode* curr);
  virtual bool generateRoot();

  //conflicts
  void findConflicts(CBSNode& curr);
  void findConflicts(CBSNode& curr, int a1, int a2);
  shared_ptr<Conflict> chooseConflict(const CBSNode& node) const;
  void classifyConflicts(CBSNode& parent);
  // void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
  //  list<shared_ptr<Conflict>>& copy, int excluded_agent) const;
  void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
             list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agent) const;
  void removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const;
  //bool isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node);

  void computePriorityForConflict(Conflict& conflict, CBSNode& node);

  //update information
  inline void updatePaths(CBSNode* curr);
  void updateFocalList();
  inline void releaseNodes();
  //inline void releaseMDDTable();
  // void copyConflictGraph(CBSNode& child, const CBSNode& parent);

  // print and save
  void printPaths() const;
  void printResults() const;
  void printConflicts(const CBSNode& curr) const;

  bool validateSolution() const;
  inline int getAgentLocation(int agent_id, size_t timestep) const;
  inline void pushNode(CBSNode* node);
};
