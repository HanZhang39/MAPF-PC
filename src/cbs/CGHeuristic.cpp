#include "CBSHeuristic.h"

int CGHeuristic::minimumVertexCover(const std::vector<int>& CG, int old_mvc, int cols, int num_of_CGedges)
{
	clock_t t = clock();
	int rst = 0;
	if (num_of_CGedges < 2)
		return num_of_CGedges;
	// Compute #CG nodes that have edges
	int num_of_CGnodes = 0;
	for (int i = 0; i < cols; i++)
    {
      for (int j = 0; j < cols; j++)
        {
          if (CG[i * cols + j] > 0)
            {
              num_of_CGnodes++;
              break;
            }
        }
    }

	if (old_mvc == -1)
    {
      for (int i = 1; i < num_of_CGnodes; i++)
        {
          if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i, cols))
            {
              rst = i;
              break;
            }
        }
      assert(rst > 0);
    }
	else
    {
      if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc - 1, cols))
        rst = old_mvc - 1;
      else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, old_mvc, cols))
        rst = old_mvc;
      else
        rst = old_mvc + 1;
    }
	runtime_solve_MVC += (double) (clock() - t) / CLOCKS_PER_SEC;
	return rst;
}



int CGHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit){

  int h = -1;
	int num_of_CGedges = 0;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
  buildCardinalConflictGraph(curr, HG, num_of_CGedges);
  // Minimum Vertex Cover
  if (curr.parent == nullptr || num_of_CGedges > ILP_edge_threshold || // root node of CBS tree or the CG is too large
	  target_reasoning || disjoint_splitting) // when we are allowed to replan for multiple agents, the incremental method is not correct any longer.
    h = minimumVertexCover(HG);
  else
    h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
  return h;

}

void CGHeuristic::buildCardinalConflictGraph(CBSNode& curr, vector<int>& CG, int& num_of_CGedges)
{
	num_of_CGedges = 0;
	for (const auto& conflict : curr.conflicts)
    {
      if (conflict->priority == conflict_priority::CARDINAL)
        {
          int a1 = conflict->a1;
          int a2 = conflict->a2;
          if (!CG[a1 * num_of_agents + a2])
            {
              CG[a1 * num_of_agents + a2] = true;
              CG[a2 * num_of_agents + a1] = true;
              num_of_CGedges++;
            }
        }
    }
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
}
