#pragma once

#include "Instance.h"
#include "ConstraintTable.h"

class LLNode // low-level node
{
public:
	int location;
  unsigned int stage;
  unsigned int dist_to_next=0;
  vector<int> timestamps;
  vector<int> secondary_keys;

	int g_val;
	int h_val = 0;
	LLNode* parent;
	int timestep = 0;
	int num_of_conflicts = 0;
	bool in_openlist = false;
	bool wait_at_goal; // the action is to wait at the goal vertex or not. This is used for >length constraints
	// the following is used to compare nodes in the OPEN list
	struct compare_node
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
        return rand() % 2;
			// 	if (n1->h_val == n2->h_val)
			// 		return rand() % 2;
			// 	return n1->h_val >= n2->h_val;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

  struct compare_timestamps
  {
		int operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
      // compare the lex order of timestamps
      int j = min(n1->timestamps.size(), n2->timestamps.size());
      for (int i = 0; i < j; i++){
        if (n1->timestamps[i] != n2->timestamps[i]){
          return n1->timestamps[i] > n2->timestamps[i]?1:-1;
        }
      }
      int last_val_1;
      int last_val_2;

      if (n1->timestamps.size() > n2->timestamps.size()){
        last_val_1 = n1->timestamps[n2->timestamps.size()];
        last_val_2 = n2->g_val + n2->dist_to_next;
      }
      if (n1->timestamps.size() < n2->timestamps.size()){
        last_val_2 = n2->timestamps[n1->timestamps.size()];
        last_val_1 = n1->g_val + n1->dist_to_next;
      }
      if (n1->timestamps.size() == n2->timestamps.size()){
        last_val_2 = n2->g_val + n2->dist_to_next;
        last_val_1 = n1->g_val + n1->dist_to_next;
      }

      if (last_val_1 != last_val_2){
        return last_val_1 > last_val_2?1:-1;
      }

      return 0;
    }
  };

	// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
      for (int i = 0; i < min(n1->secondary_keys.size(), n2->secondary_keys.size()); i++ ){
        if (n1->secondary_keys[i] != n2->secondary_keys[i]){
          return n1->secondary_keys[i] > n2->secondary_keys[i];
        }
      }
      // TODO comparing timestamps can be converted to secondary key as well.
      auto res = compare_timestamps()(n1,n2);
      if (res != 0){
        return res == 1? true: false;
      }
			if (n1->num_of_conflicts == n2->num_of_conflicts)
			{
				if (n1->g_val == n2->g_val)
				{
					return rand() % 2 == 0;
				}
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			}
			return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)

	struct secondary_compare_node_not_random
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
      auto res = compare_timestamps()(n1,n2);
      if (res != 0){
        return res == 1? true: false;
      }
			if (n1->num_of_conflicts == n2->num_of_conflicts)
        {
          if (n1->g_val == n2->g_val)
            {
              return false;
            }
          return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
        }
			return n1->num_of_conflicts > n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
		}
	};  

	LLNode() : location(0), stage(0), g_val(0), h_val(0), parent(nullptr), timestep(0), num_of_conflicts(0), in_openlist(false), wait_at_goal(false) {}
  virtual ~LLNode(){};

	LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep, unsigned int stage, int num_of_conflicts = 0, bool in_openlist = false) :
		location(location), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep), stage(stage),
		num_of_conflicts(num_of_conflicts), in_openlist(in_openlist), wait_at_goal(false) {}

	inline double getFVal() const { return g_val + h_val; }

	void copy(const LLNode& other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
		num_of_conflicts = other.num_of_conflicts;
		wait_at_goal = other.wait_at_goal;
	}
};


class SingleAgentSolver
{
public:
	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;

	double runtime_build_CT = 0; // runtime of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table

	int start_location;
	vector<int> goal_location;
  vector<int> heuristic_landmark;
	vector<vector<int>> my_heuristic;  // this is the precomputed heuristic for this agent
  int get_heuristic(int stage, int loc) const{
    // h to next goal and h from next goal to the last goal
    return my_heuristic[stage][loc] + heuristic_landmark[stage];
  }
	int compute_heuristic(int from, int to) const  // compute admissible heuristic between two locations
	{
		return max(get_DH_heuristic(from, to), instance.getManhattanDistance(from, to));
	}

	const Instance& instance;

	virtual Path findPath(const CBSNode& node, const ConstraintTable& initial_constraints,
						  const vector<Path*>& paths, int agent, int lower_bound) = 0;
	virtual Path findPathSegment(ConstraintTable& constraint_table, int start_time, int stage, int lowerbound) = 0;
	virtual int getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound) = 0;
	virtual string getName() const = 0;

	list<int> getNextLocations(int curr) const; // including itself and its neighbors
	list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }

	// int getStartLocation() const {return instance.start_locations[agent]; }
	// int getGoalLocation() const {return instance.goal_locations[agent]; }

	SingleAgentSolver(const Instance& instance, int agent) :
		instance(instance), //agent(agent), 
		start_location(instance.start_locations[agent]),
		goal_location(instance.goal_locations[agent])
	{
		compute_heuristics();
	}

	virtual ~SingleAgentSolver() {}

  bool use_timestamps = true;


protected:
	void compute_heuristics();

	int get_DH_heuristic(int from, int to) const { return abs(my_heuristic[0][from] - my_heuristic[0][to]); }
};
