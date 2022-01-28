#pragma once

#include "ReservationTable.h"
#include "Instance.h"
#include "SingleAgentSolver.h"


class CorridorReasoning
{
public:
	bool use_corridor_reasoning;
	double accumulated_runtime = 0;

	CorridorReasoning(const vector<SingleAgentSolver*>& search_engines,
					  const vector<ConstraintTable>& initial_constraints) :
			search_engines(search_engines), initial_constraints(initial_constraints) {}

	shared_ptr<Conflict> run(const shared_ptr<Conflict>& conflict,
							 const vector<Path*>& paths,
							 bool cardinal, const CBSNode& node);

private:
	const vector<SingleAgentSolver*>& search_engines;
	const vector<ConstraintTable>& initial_constraints;

	shared_ptr<Conflict> findCorridorConflict(const shared_ptr<Conflict>& conflict,
											  const vector<Path*>& paths,
											  bool cardinal, const CBSNode& node);
	int findCorridor(const shared_ptr<Conflict>& conflict,
					 const vector<Path*>& paths, int endpoints[], int endpoints_time[]); // return the length of the corridor
	int getEnteringTime(const Path& path, const Path& path2, int t);
	int getExitingTime(const Path& path, int t);
	int getCorridorLength(const Path& path, int t_start, int loc_end, std::pair<int, int>& edge);


	// int getBypassLength(int start, int end, std::pair<int, int> blocked, const bool* my_map, int num_col, int map_size);
	// int getBypassLengthByAStar(int start, int end, std::pair<int, int> blocked,
	//	const ConstraintTable& constraint_table, int upper_bound);
	// int getBypassLengthBySIPP(int start, int end, std::pair<int, int> blocked,
	//	 ReservationTable& reservation_table, int upper_bound);

	bool blocked(const Path& path, const Constraint& constraint);

};

