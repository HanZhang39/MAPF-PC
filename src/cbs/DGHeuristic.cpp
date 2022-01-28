#include "CBSHeuristic.h"

int DGHeuristic::computeInformedHeuristicsValue(CBSNode& curr, double time_limit){
  int h = -1;
	int num_of_CGedges;
	vector<int> HG(num_of_agents * num_of_agents, 0); // heuristic graph
  if (!buildDependenceGraph(curr, HG, num_of_CGedges))
    return false;
  // Minimum Vertex Cover
  if (curr.parent == nullptr || num_of_CGedges > ILP_edge_threshold ||// root node of CBS tree or the CG is too large
      target_reasoning || disjoint_splitting) // when we are allowed to replan for multiple agents, the incremental method is not correct any longer.
    h = minimumVertexCover(HG);
  else
    h = minimumVertexCover(HG, curr.parent->h_val, num_of_agents, num_of_CGedges);
  return h;
}


bool DGHeuristic::buildDependenceGraph(CBSNode& node, vector<int>& CG, int& num_of_CGedges)
{
	for (auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		if (conflict->priority == conflict_priority::CARDINAL)
		{
			node.conflictGraph[idx] = 1;
		}
		else if (node.conflictGraph.find(idx) == node.conflictGraph.end())
		{
			auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
			if (got != lookupTable[a1][a2].end()) // check the lookup table first
			{
				num_memoization++;
				node.conflictGraph[idx] = got->second;
			}
			else
			{
				node.conflictGraph[idx] = dependent(a1, a2, node) ? 1 : 0;
				lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = node.conflictGraph[idx];
			}
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

	num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = i + 1; j < num_of_agents; j++)
		{
			auto got = node.conflictGraph.find(i * num_of_agents + j);
			if (got != node.conflictGraph.end() && got->second > 0)
			{
				CG[i * num_of_agents + j] = got->second;
				CG[j * num_of_agents + i] = got->second;
				num_of_CGedges++;
			}
		}
	}
	runtime_build_dependency_graph += (double) (clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}


// int DGHeuristic::getEdgeWeight(int a1, int a2, CBSNode& node, bool cardinal)
// {
// 	HTableEntry newEntry(a1, a2, &node);
//   HTable::const_iterator got = lookupTable[a1][a2].find(newEntry);

//   if (got != lookupTable[a1][a2].end())
// 		{
// 			num_memoization++;
// 			return got->second;
// 		}


// 	// runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
// 	if (screen > 2)
// 	{
// 		cout << "Agents " << a1 << " and " << a2 << " in node " << node.time_generated << " : ";
// 	}
// 	int rst = 0;
// 	if (cardinal)
// 		rst = 1;
// 	else if (!mutex_reasoning) // no mutex reasoning, so we might miss some cardinal conflicts
// 	{
//     // merging MDDs
//     rst = dependent(a1, a2, node)? 0 : 1;
// 	}
// 	lookupTable[a1][a2][newEntry] = rst;
// 	return rst;
// }

bool DGHeuristic::dependent(int a1, int a2, CBSNode& node) // return true if the two agents are dependent
{
	const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size()); // get mdds
	const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
	if (mdd1->levels.size() > mdd2->levels.size()) // swap
		std::swap(mdd1, mdd2);
	num_merge_MDDs++;
	return !SyncMDDs(*mdd1, *mdd2);
}

bool DGHeuristic::SyncMDDs(const MDD &mdd, const MDD& other) // assume mdd.levels <= other.levels
{
	if (other.levels.size() <= 1) // Either of the MDDs was already completely pruned already
		return false;

	SyncMDD copy(mdd);
	if (copy.levels.size() < other.levels.size())
	{
		size_t i = copy.levels.size();
		copy.levels.resize(other.levels.size());
		for (; i < copy.levels.size(); i++)
		{
			SyncMDDNode* parent = copy.levels[i - 1].front();
			auto node = new SyncMDDNode(parent->location, parent);
			parent->children.push_back(node);
			copy.levels[i].push_back(node);

		}
	}
	// Cheaply find the coexisting nodes on level zero - all nodes coexist because agent starting points never collide
	copy.levels[0].front()->coexistingNodesFromOtherMdds.push_back(other.levels[0].front());

	// what if level.size() = 1?
	for (size_t i = 1; i < copy.levels.size(); i++)
	{
		for (auto node = copy.levels[i].begin(); node != copy.levels[i].end();)
		{
			// Go over all the node's parents and test their coexisting nodes' children for co-existance with this node
			for (auto parent = (*node)->parents.begin(); parent != (*node)->parents.end(); parent++)
			{
				//bool validParent = false;
				for (const MDDNode* parentCoexistingNode : (*parent)->coexistingNodesFromOtherMdds)
				{
					for (const MDDNode* childOfParentCoexistingNode : parentCoexistingNode->children)
					{
						if ((*node)->location == childOfParentCoexistingNode->location) // vertex conflict
							continue;
						else if ((*node)->location == parentCoexistingNode->location && (*parent)->location == childOfParentCoexistingNode->location) // edge conflict
							continue;
						//validParent = true;

						auto it = (*node)->coexistingNodesFromOtherMdds.cbegin();
						for (; it != (*node)->coexistingNodesFromOtherMdds.cend(); ++it)
						{
							if (*it == childOfParentCoexistingNode)
								break;
						}
						if (it == (*node)->coexistingNodesFromOtherMdds.cend())
						{
							(*node)->coexistingNodesFromOtherMdds.push_back(childOfParentCoexistingNode);
						}
					}
				}
				//if (!validParent)
				//{
				//	// delete the edge, and continue up the levels if necessary
				//	SyncMDDNode* p = *parent;
				//	parent = (*node)->parents.erase(parent);
				//	p->children.remove((*node));
				//	if (p->children.empty())
				//		copy.deleteNode(p);
				//}
				//else
				//{
				//	parent++;
				//}
			}
			if ((*node)->coexistingNodesFromOtherMdds.empty())
			{
				// delete the node, and continue up the levels if necessary
				SyncMDDNode* p = *node;
				node++;
				copy.deleteNode(p, i);
			}
			else
				node++;
		}
		if (copy.levels[i].empty())
		{
			copy.clear();
			return false;
		}
	}
	copy.clear();
	return true;
}

void DGHeuristic::copyConflictGraph(CBSNode& child, const CBSNode& parent)
{
  unordered_set<int> changed;
  for (const auto& p : child.paths) {
    changed.insert(p.first);
  }
  for (auto e : parent.conflictGraph) {
    if (changed.find(e.first / num_of_agents) == changed.end() &&
				changed.find(e.first % num_of_agents) == changed.end())
      child.conflictGraph[e.first] = e.second;

	}
}


