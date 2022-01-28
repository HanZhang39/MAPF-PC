#include "CBSHeuristic.h"

int ZeroHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit)
{
  return 0;
}


bool CBSHeuristic::computeInformedHeuristics(CBSNode& curr, double time_limit){
  curr.h_computed =true;
	start_time = clock();
	this->time_limit = time_limit;

  int h = computeInformedHeuristicsValue(curr, time_limit);
	if (h < 0)
		return false;

	curr.h_val = max(h, curr.h_val);

	// update tie-breaking for node selection if necessary
	if (node_selection_rule == node_selection::NODE_H)
		curr.tie_breaking = curr.h_val;

	return true;
}


void CBSHeuristic::copyConflictGraph(CBSNode& child, const CBSNode& parent)
{
	//copy conflict graph
  // Do nothing
}

bool CBSHeuristic::shouldEvalHeuristic(CBSNode* node)
{
  return !node->h_computed;
}

void CBSHeuristic::computeQuickHeuristics(CBSNode& node) // for non-root node
{
	node.h_val = max(0, node.parent->g_val + node.parent->h_val - node.g_val); // pathmax
	set<pair<int, int>> conflicting_agents;
	switch (node_selection_rule)
	{
	case node_selection::NODE_H:
		node.tie_breaking = node.h_val;
		break;
	case node_selection::NODE_DEPTH:
		node.tie_breaking = -node.depth; // we use negative depth because we prefer nodes with larger depths
		break;
	case node_selection::NODE_CONFLICTS:
		node.tie_breaking = (int) (node.conflicts.size() + node.unknownConf.size());
		break;
	case node_selection::NODE_CONFLICTPAIRS:
		for (const auto& conflict : node.unknownConf)
		{
			auto agents = make_pair(min(conflict->a1, conflict->a2), max(conflict->a1, conflict->a2));
			if (conflicting_agents.find(agents) == conflicting_agents.end())
			{
				conflicting_agents.insert(agents);
			}
		}
		node.tie_breaking = (int) (node.conflicts.size() + conflicting_agents.size());
		break;
	case node_selection::NODE_MVC:
		node.tie_breaking = MVConAllConflicts(node);
		break;
  case node_selection::NODE_RANDOM:
    break;
	}
	copyConflictGraph(node, *node.parent);
}

int CBSHeuristic::MVConAllConflicts(CBSNode& curr)
{
	auto G = buildConflictGraph(curr);
	return minimumVertexCover(G);
}

int CBSHeuristic::minimumVertexCover(const vector<int>& CG)
{
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> indices;
		indices.reserve(num_of_agents);
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (CG[j * num_of_agents + k] > 0)
				{
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
				else if (CG[k * num_of_agents + j] > 0)
				{
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
		}
		if ((int) indices.size() == 1) //one node -> no edges -> mvc = 0
			continue;
		else if ((int) indices.size() == 2) // two nodes -> only one edge -> mvc = 1
		{
			rst += 1; // add edge weight
			continue;
		}

		std::vector<int> subgraph(indices.size() * indices.size(), 0);
		int num_edges = 0;
		for (int j = 0; j < (int) indices.size(); j++)
		{
			for (int k = j + 1; k < (int) indices.size(); k++)
			{
				subgraph[j * indices.size() + k] = CG[indices[j] * num_of_agents + indices[k]];
				subgraph[k * indices.size() + j] = CG[indices[k] * num_of_agents + indices[j]];
				if (subgraph[j * indices.size() + k] > 0)
					num_edges++;
			}
		}
		if (num_edges > ILP_edge_threshold)
		{
			vector<int> ranges(indices.size(), 1);
			rst += ILPForWMVC(subgraph, ranges);
			double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
			if (runtime > time_limit)
				return -1; // run out of time
		}
		else
		{
			for (int i = 1; i < (int) indices.size(); i++)
			{
				if (KVertexCover(subgraph, (int) indices.size(), num_edges, i, (int) indices.size()))
				{
					rst += i;
					break;
				}
				double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
				if (runtime > time_limit)
					return -1; // run out of time
			}
		}
	}
	return rst;
}

// Whether there exists a k-vertex cover solution
bool CBSHeuristic::KVertexCover(const std::vector<int>& CG, int num_of_CGnodes, int num_of_CGedges, int k, int cols)
{
	double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return true; // run out of time
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k)
		return false;

	std::vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < cols - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < cols && flag; j++)
		{
			if (CG[i * cols + j] > 0)
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		std::vector<int> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < cols; j++)
		{
			if (CG_copy[node[i] * cols + j] > 0)
			{
				CG_copy[node[i] * cols + j] = 0;
				CG_copy[j * cols + node[i]] = 0;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1, cols))
			return true;
	}
	return false;
}


vector<int> CBSHeuristic::buildConflictGraph(const CBSNode& curr) const
{
	vector<int> G(num_of_agents * num_of_agents, 0);
	for (const auto& conflict : curr.conflicts)
    {
      int a1 = conflict->a1;
      int a2 = conflict->a2;
      if (!G[a1 * num_of_agents + a2])
        {
          G[a1 * num_of_agents + a2] = true;
          G[a2 * num_of_agents + a1] = true;
        }
    }
	return G;
}

int CBSHeuristic::greedyMatching(const std::vector<int>& CG, int cols)
{
	int rst = 0;
	std::vector<bool> used(cols, false);
	while (1)
    {
      int maxWeight = 0;
      int ep1, ep2;
      for (int i = 0; i < cols; i++)
        {
          if (used[i])
            continue;
          for (int j = i + 1; j < cols; j++)
            {
              if (used[j])
                continue;
              else if (maxWeight < CG[i * cols + j])
                {
                  maxWeight = CG[i * cols + j];
                  ep1 = i;
                  ep2 = j;
                }
            }
        }
      if (maxWeight == 0)
        return rst;
      rst += maxWeight;
      used[ep1] = true;
      used[ep2] = true;
    }
}
