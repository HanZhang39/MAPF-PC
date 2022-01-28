#pragma once

#include "MDD.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "MutexReasoning.h"

enum heuristics_type { ZERO, CG, DG, WDG, STRATEGY_COUNT };


struct HTableEntry // look-up table entry 
{
	int a1{};
	int a2{};
	CBSNode* n{};

	HTableEntry() = default;
	HTableEntry(int a1, int a2, CBSNode* n) : a1(a1), a2(a2), n(n) {};

	struct EqNode
	{
		bool operator() (const HTableEntry& h1, const HTableEntry& h2) const
		{
			std::set<Constraint> cons1[2], cons2[2];
			const CBSNode* curr = h1.n;
			while (curr->parent != nullptr)
			{
				if (std::get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
          {
            for (auto con : curr->constraints)
              {
                cons1[0].insert(con);
                cons2[0].insert(con);
              }
          }
				else
          {
            if (std::get<0>(curr->constraints.front()) == h1.a1)
              for (auto con : curr->constraints)
                cons1[0].insert(con);
            else if (std::get<0>(curr->constraints.front()) == h1.a2)
              for (auto con : curr->constraints)
                cons2[0].insert(con);
          }

				curr = curr->parent;
			}
			curr = h2.n;
			while (curr->parent != nullptr)
        {
          if (std::get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
              std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
              std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE) {
            for (auto con : curr->constraints)
              {
                cons1[1].insert(con);
                cons2[1].insert(con);
              }
          }
          else {
            if (std::get<0>(curr->constraints.front()) == h2.a1)
              for (auto con : curr->constraints)
                cons1[1].insert(con);
            else if (std::get<0>(curr->constraints.front()) == h2.a2)
              for (auto con : curr->constraints)
                cons2[1].insert(con);
          }

          curr = curr->parent;
        }
			if (cons1[0].size() != cons1[1].size() || cons2[0].size() != cons2[1].size())
				return false;

			if (!equal(cons1[0].begin(), cons1[0].end(), cons1[1].begin()))
				return false;
			return equal(cons2[0].begin(), cons2[0].end(), cons2[1].begin());
		}
	};


	struct Hasher
	{
		size_t operator()(const HTableEntry& entry) const
		{
			CBSNode* curr = entry.n;
			size_t cons1_hash = 0, cons2_hash = 0;
			while (curr->parent != nullptr)
			{
				if (std::get<0>(curr->constraints.front()) == entry.a1 ||
					std::get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons1_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				else if (std::get<0>(curr->constraints.front()) == entry.a2 ||
					std::get<4>(curr->constraints.front()) == constraint_type::LEQLENGTH ||
					std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_VERTEX ||
					std::get<4>(curr->constraints.front()) == constraint_type::POSITIVE_EDGE)
				{
					for (auto con : curr->constraints)
					{
						cons2_hash += 3 * std::hash<int>()(std::get<0>(con)) +
							5 * std::hash<int>()(std::get<1>(con)) +
							7 * std::hash<int>()(std::get<2>(con)) +
							11 * std::hash<int>()(std::get<3>(con));
					}
				}
				curr = curr->parent;
			}
			return cons1_hash ^ (cons2_hash << 1);
		}
	};
};


typedef unordered_map<HTableEntry, int, HTableEntry::Hasher, HTableEntry::EqNode> HTable;

class CBSHeuristic
{
public:
	bool rectangle_reasoning; // using rectangle reasoning
	bool corridor_reasoning; // using corridor reasoning
	bool target_reasoning; // using target reasoning
	mutex_strategy mutex_reasoning; // using mutex reasoning
	bool disjoint_splitting; // disjoint splittting
	bool PC; // prioritize conflicts
	conflict_selection conflict_seletion_rule;
	node_selection node_selection_rule;

	double runtime_build_dependency_graph = 0;
	double runtime_solve_MVC = 0;

	uint64_t num_merge_MDDs = 0;
	uint64_t num_solve_2agent_problems = 0;
	uint64_t num_memoization = 0; // number of times when memeorization helps

	CBSHeuristic(int num_of_agents,
               const vector<Path*>& paths,
               vector<SingleAgentSolver*>& search_engines,
               const vector<ConstraintTable>& initial_constraints,
               MDDTable& mdd_helper) : num_of_agents(num_of_agents),
                                       paths(paths), search_engines(search_engines), initial_constraints(initial_constraints), mdd_helper(mdd_helper) {}

  virtual void copyConflictGraph(CBSNode& child, const CBSNode& parent);
	bool computeInformedHeuristics(CBSNode& curr, double time_limit);
	virtual void computeQuickHeuristics(CBSNode& curr); // this function is called when generating a CT node

	virtual void init() {}
	virtual void clear() {}
  bool shouldEvalHeuristic(CBSNode* node);
  virtual heuristics_type getType() const =0;

protected:
	int screen = 0;
	int num_of_agents;
	int ILP_node_threshold = 5; // run ILP if #nodes in the conrflict graph is larger than the threshold
	int ILP_edge_threshold = 10; // run ILP if #edges in the conrflict graph is larger than the threshold

	double time_limit;
	int node_limit = 64;  // terminate the sub CBS solver if the number of its expanded nodes exceeds the node limit.
	double start_time;

	const vector<Path*>& paths;
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;
	MDDTable& mdd_helper;

	int MVConAllConflicts(CBSNode& curr);
	vector<int> buildConflictGraph(const CBSNode& curr) const;
  int greedyMatching(const std::vector<int>& CG, int cols);
  int minimumVertexCover(const vector<int>& CG);
	virtual bool KVertexCover(const vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols);
	int ILPForWMVC(const vector<int>& CG, const vector<int>& node_max_value);
	virtual int computeInformedHeuristicsValue(CBSNode& curr, double time_limit)=0;

};


class ZeroHeuristic: public CBSHeuristic {
public:
	ZeroHeuristic(int num_of_agents,
               const vector<Path*>& paths,
               vector<SingleAgentSolver*>& search_engines,
               const vector<ConstraintTable>& initial_constraints,
                MDDTable& mdd_helper) : CBSHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper)
  {
  }
  virtual heuristics_type getType() const
  {
    return heuristics_type::ZERO;
  }

protected:
  virtual int computeInformedHeuristicsValue(CBSNode& curr, double time_limit);
};

class CGHeuristic: public CBSHeuristic {
public:
	CGHeuristic(int num_of_agents,
                const vector<Path*>& paths,
                vector<SingleAgentSolver*>& search_engines,
                const vector<ConstraintTable>& initial_constraints,
                MDDTable& mdd_helper) : CBSHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper)
  {
  }

  virtual heuristics_type getType() const
  {
    return heuristics_type::CG;
  }

protected:
	virtual int computeInformedHeuristicsValue(CBSNode& curr, double time_limit);
  virtual void buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges);
  using CBSHeuristic::minimumVertexCover;
  virtual int minimumVertexCover(const vector<int>& CG, int old_mvc, int cols, int num_of_edges);
};


class DGHeuristic: public CGHeuristic {
public:
	DGHeuristic(int num_of_agents,
              const vector<Path*>& paths,
              vector<SingleAgentSolver*>& search_engines,
              const vector<ConstraintTable>& initial_constraints,
              MDDTable& mdd_helper) : CGHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper)
  {
  }

  virtual heuristics_type getType() const 
  {
    return heuristics_type::DG;
  }

	virtual void init()
	{
    lookupTable.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
      {
        lookupTable[i].resize(num_of_agents);
      }
	}
  virtual void copyConflictGraph(CBSNode& child, const CBSNode& parent);

  virtual void clear() { lookupTable.clear(); }
protected:
	vector<vector<HTable> > lookupTable;

	virtual int computeInformedHeuristicsValue(CBSNode& curr, double time_limit);
	virtual bool buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges);
	// virtual int getEdgeWeight(int a1, int a2, CBSNode& node, bool cardinal);
	virtual bool SyncMDDs(const MDD &mdd1, const MDD& mdd2);
  virtual bool dependent(int a1, int a2, CBSNode& node);
};

class WDGHeuristic: public DGHeuristic {
public:
	WDGHeuristic(int num_of_agents,
                const vector<Path*>& paths,
                vector<SingleAgentSolver*>& search_engines,
                const vector<ConstraintTable>& initial_constraints,
                MDDTable& mdd_helper) : DGHeuristic(num_of_agents, paths, search_engines, initial_constraints, mdd_helper)
  {
  }

  virtual heuristics_type getType() const
  {
    return heuristics_type::WDG;
  }

protected:
	virtual int computeInformedHeuristicsValue(CBSNode& curr, double time_limit);
	virtual bool buildWeightedDependenceGraph(CBSNode& node, vector<int>& CG);
	int solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal);
	int DPForWMVC(vector<int>& x, int i, int sum, const vector<int>& CG, const vector<int>& range, int& best_so_far);
	int minimumWeightedVertexCover(const vector<int>& CG);
	int weightedVertexCover(const vector<int>& CG);
};


