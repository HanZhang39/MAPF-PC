#include "CBSHeuristic.h"
#include "CBS.h"


int WDGHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit)
{
  int h = -1;
	int num_of_CGedges;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
  if (!buildWeightedDependenceGraph(curr, HG))
    return false;
  h = minimumWeightedVertexCover(HG);
  return h;
}

int WDGHeuristic::minimumWeightedVertexCover(const vector<int>& HG)
{
	clock_t t = clock();
	int rst = weightedVertexCover(HG);
	runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

int WDGHeuristic::weightedVertexCover(const std::vector<int>& CG)
{
int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], CG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
				else if (CG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], CG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(CG[indices[0] * num_of_agents + indices[1]], CG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(CG[indices[j] * num_of_agents + indices[k]], CG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		if (num > ILP_node_threshold) // solve by ILP
		{
			rst += ILPForWMVC(G, range);
		}
		else // solve by dynamic programming
		{
			std::vector<int> x(num);
			int best_so_far = MAX_COST;
			rst += DPForWMVC(x, 0, 0, G, range, best_so_far);
		}
		double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}

	//test
	/*std::vector<int> x(N, 0);
	std::vector<int> range(N, 0);
	for (int i = 0; i < N; i++)
	{
		for (int j = i + 1; j < N; j++)
		{
			range[i] = std::max(range[i], CG[i * N + j]);
			range[j] = std::max(range[j], CG[i * N + j]);
		}
	}
	int best_so_far = INT_MAX;
	int rst2 = DPForWMVC(x, 0, 0, CG, range, best_so_far);
	if ( rst != rst2)
		std::cout << "ERROR" <<std::endl;*/

	return rst;
}

bool WDGHeuristic::buildWeightedDependenceGraph(CBSNode& node, vector<int>& CG)
{
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (node.conflictGraph.find(idx) != node.conflictGraph.end())
			continue;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;
			node.conflictGraph[idx] = got->second;
		}
		else if (rectangle_reasoning || mutex_reasoning)
		{
			node.conflictGraph[idx] = solve2Agents(a1, a2, node, false);
			assert(node.conflictGraph[idx] >= 0);
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.conflictGraph[idx];
		}
		else
		{
			bool cardinal = conflict->priority == conflict_priority::CARDINAL;
			if (!cardinal) // using merging MDD methods before running 2-agent instance
			{
				cardinal = dependent(a1, a2, node);
			}
			if (cardinal) // run 2-agent solver only for dependent agents
			{
				node.conflictGraph[idx] = solve2Agents(a1, a2, node, cardinal);
				assert(node.conflictGraph[idx] >= 1);
			}
			else
			{
				node.conflictGraph[idx] = 0;
			}
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.conflictGraph[idx];
		}

		if (node.conflictGraph[idx] == MAX_COST) // no solution
		{
			return false;
		}

		if (conflict->priority != conflict_priority::CARDINAL && node.conflictGraph[idx] > 0)
		{
			conflict->priority = conflict_priority::PSEUDO_CARDINAL; // the two agents are dependent, although resolving this conflict might not increase the cost
		}
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_dependency_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
	}

	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = node.conflictGraph.find(i * num_of_agents + j);
			if (got != node.conflictGraph.end() && got->second > 0)
			{
				CG[i * num_of_agents + j] = got->second;
				CG[j * num_of_agents + i] = got->second;
			}
		}
	}
	runtime_build_dependency_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


// recursive component of dynamic programming for weighted vertex cover
int
WDGHeuristic::DPForWMVC(std::vector<int>& x, int i, int sum, const std::vector<int>& CG, const std::vector<int>& range,
						int& best_so_far)
{
	if (sum >= best_so_far)
		return MAX_COST;
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int) x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForWMVC(x, i + 1, sum, CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}

	int cols = x.size();

	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
		{
			min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = DPForWMVC(x, i + 1, sum + x[i], CG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost;
	}

	return best_so_far;
}

int WDGHeuristic::solve2Agents(int a1, int a2, const CBSNode& node, bool cardinal)
{
	vector<SingleAgentSolver*> engines(2);
	engines[0] = search_engines[a1];
	engines[1] = search_engines[a2];
	vector<Path> initial_paths(2);
	initial_paths[0] = *paths[a1];
	initial_paths[1] = *paths[a2];
	vector<ConstraintTable> constraints{
			ConstraintTable(initial_constraints[a1]),
			ConstraintTable(initial_constraints[a2]) };
	constraints[0].build(node, a1, search_engines[a1]->goal_location.size());
	constraints[1].build(node, a2, search_engines[a2]->goal_location.size());
	CBS cbs(engines, constraints, initial_paths, heuristics_type::CG, screen);
	cbs.setPrioritizeConflicts(PC);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_seletion_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setNodeLimit(node_limit);

	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	int root_g = (int) initial_paths[0].size() - 1 + (int) initial_paths[1].size() - 1;
	int lowerbound = root_g;
	int upperbound = MAX_COST;
	if (cardinal)
		lowerbound += 1;
	cbs.solve(time_limit - runtime, lowerbound, upperbound);
	num_solve_2agent_problems++;
	int rst;
	if (cbs.runtime > time_limit - runtime || cbs.num_HL_expanded > node_limit) // time out or node out
		rst = (int) cbs.min_f_val - root_g; // using lowerbound to approximate
	else if (cbs.solution_cost < 0) // no solution
		rst = MAX_COST;
	else
	{
		rst = cbs.solution_cost - root_g;
	}
	assert(rst >= 0);
	return rst;
}
