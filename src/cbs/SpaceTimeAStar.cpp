#include "SpaceTimeAStar.h"

void MultiLabelSpaceTimeAStar::updatePath(const LLNode* goal, Path& path)
{
	path.path.resize(goal->g_val + 1);
	path.timestamps.resize(goal_location.size(), 0);
  path.timestamps.back() = goal->g_val;

	const LLNode* curr = goal;
	while (curr != nullptr) 
	{
		path[curr->g_val].location = curr->location;
		// path[curr->g_val].single = false;
		path[curr->g_val].mdd_width = 0;

    if (curr->parent !=nullptr && curr->stage != curr->parent->stage){
      path.timestamps[curr->parent->stage] = curr->g_val;
      path[curr->g_val].is_goal = true;
    }else{
      path[curr->g_val].is_goal = false;
    }

		curr = curr->parent;
	}
}


Path MultiLabelSpaceTimeAStar::findPath(const CBSNode& node, const ConstraintTable& initial_constraints,
							  const vector<Path*>& paths, int agent, int lowerbound)
{
	num_expanded = 0;
	num_generated = 0;
	// build constraint table
	auto starrt_time = clock();
	ConstraintTable constraint_table(initial_constraints);
	constraint_table.build(node, agent,goal_location.size());

	runtime_build_CT = (double) (clock() - starrt_time) / CLOCKS_PER_SEC;
	if (constraint_table.length_min >= MAX_TIMESTEP || constraint_table.length_min > constraint_table.length_max ||  // the agent cannot reach
																													 // its goal location
		constraint_table.constrained(start_location, 0)) // the agent cannot stay at its start location
	{
		return Path();
	}

	starrt_time = clock();
	constraint_table.buildCAT(agent, paths, node.makespan + 1);
	runtime_build_CAT = (double) (clock() - starrt_time) / CLOCKS_PER_SEC;

    return findShortestPath(constraint_table, make_pair(start_location, 0), lowerbound);

  
}


Path MultiLabelSpaceTimeAStar::findShortestPath(ConstraintTable& constraint_table, const pair<int, int> start_state, int lowerbound)
{

  // for vertex of different stage, the f val should be bounded within the ub.
  vector<int> f_ub(goal_location.size(), INT_MAX);
  if (constraint_table.leq_goal_time[goal_location.size() - 1] != INT_MAX){
    f_ub.back() = constraint_table.leq_goal_time[goal_location.size() - 1];
  }
  for (int i = (int) goal_location.size() - 2; i >= 0; i--){
    if (constraint_table.leq_goal_time[i] != INT_MAX){
      f_ub[i] = min(f_ub[i + 1], constraint_table.leq_goal_time[i] + heuristic_landmark[i]);
    }else{
      f_ub[i] = f_ub[i + 1];
    }
  }

	// generate start and add it to the OPEN & FOCAL list
	Path path;
	auto start = new MultiLabelAStarNode(start_state.first,  // location
                                       0,  // g val 
                                       get_heuristic(0, start_state.first),  // h val
                                       nullptr,  // parent
                                       start_state.second,  // timestep
                                       0, // stage
                                       0, false);

  // start->timestamps.resize(goal_location.size());

	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;


  if (start->location==  (unsigned int)goal_location[0] &&  constraint_table.g_goal_time[0] < start->g_val){
    start->stage += 1;
    if (use_timestamps){
      // timestamps.push_back(0);
    }
  }

	allNodes_table.insert(start);
	min_f_val = (int) start->getFVal();
	int holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
	lower_bound = max(holding_time - start_state.second, max(min_f_val, lowerbound));

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto* curr = popNode();

		// check if the popped node is a goal
		if (curr->location == goal_location.back() && // arrive at the goal location
        curr->stage == goal_location.size() - 1 && // reach all previous goals
			!curr->wait_at_goal && // not wait at the goal location
			curr->timestep >= holding_time) // the agent can hold the goal location afterward
		{
			updatePath(curr, path);
			break;
		}

		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
    // generate child
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
				if (next_location == curr->location)
				{
					continue;
				}
				next_timestep--;
			}

			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

      // setting the stage
      auto stage = curr->stage;
      auto timestamps = curr->timestamps;


			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			// int next_h_val = my_heuristic[next_location];
			int next_h_val = get_heuristic(stage, next_location);
			if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
				continue;

      if (next_location ==  (unsigned int)goal_location[stage] && stage < goal_location.size() - 1 && constraint_table.g_goal_time[stage] < curr->g_val + 1){
        stage += 1;
        if (use_timestamps){
          timestamps.push_back(curr->g_val + 1);
        }
      }


			int next_internal_conflicts = curr->num_of_conflicts +
										  constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
                                curr, next_timestep, stage, next_internal_conflicts, false);

      next->timestamps  = timestamps;
      next->dist_to_next = my_heuristic[stage][next_location];

			if (next->stage == goal_location.size() - 1 && next_location == goal_location.back() && curr->location == goal_location.back()){
				next->wait_at_goal = true;
      }

			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				pushNode(next);
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == next->getFVal() &&
         LLNode::secondary_compare_node_not_random()(existing_next, next)
				 // existing_next->num_of_conflicts > next->num_of_conflicts
         )) // or it remains the same but there's fewer conflicts
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
				}
				else
				{
					bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
					bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
					bool update_open = false;
					if ((next_g_val + next_h_val) <= lower_bound)
					{  // if the new f-val qualify to be in FOCAL
						if (existing_next->getFVal() > lower_bound)
							add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
						else
							update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
					}
					if (existing_next->getFVal() > next_g_val + next_h_val)
						update_open = true;

					existing_next->copy(*next);  // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle);  // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down			
				}
			}
			delete next;  // not needed anymore -- we already generated it before
		}  // end for loop that generates successors
	}  // end while loop

	releaseNodes();
	return path;
}


Path MultiLabelSpaceTimeAStar::findPathSegment(ConstraintTable& constraint_table, int start_time, int stage, int lowerbound)
{
  int loc = start_location;
  if (stage != 0){
    loc = goal_location[stage - 1];
  }


	// generate start and add it to the OPEN & FOCAL list
	Path path;
  path.begin_time = start_time;
	auto start = new MultiLabelAStarNode(loc,  // location
                                       0,  // g val 
                                       get_heuristic(stage, loc),  // h val
                                       nullptr,  // parent
                                       start_time,  // timestep
                                       stage, // stage
                                       0, false);

  start->secondary_keys.push_back(-start->g_val);
  // start->timestamps.resize(goal_location.size());

	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table.insert(start);
	min_f_val = (int) start->getFVal();
	int holding_time = constraint_table.length_min;
  if (stage == goal_location.size() - 1){
    holding_time = constraint_table.getHoldingTime(); // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
  }
	lower_bound = max(holding_time - start_time, max(min_f_val, lowerbound));

	while (!open_list.empty())
	{
		updateFocalList(); // update FOCAL if min f-val increased
		auto* curr = popNode();

		// check if the popped node is a goal
		if (curr->location == goal_location[stage] && // reach all previous goals
			curr->timestep >= holding_time) // the agent can hold the goal location afterward
		{
			updatePath(curr, path);
			break;
		}
		if (curr->timestep >= constraint_table.length_max)
			continue;

		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
    // generate child
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			if (max(constraint_table.cat_size, constraint_table.latest_timestep) + 1 < curr->timestep)
			{ // now everything is static, so switch to space A* where we always use the same timestep
				if (next_location == curr->location)
				{
					continue;
				}
				next_timestep--;
			}

			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

      // setting the stage
      auto stage = curr->stage;
      auto timestamps = curr->timestamps;

      // if (next_location ==  (unsigned int)goal_location[stage] &&  < curr->g_val + 1){
      //   stage += 1;
      //   timestamps.push_back(curr->g_val + 1);
      // }

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			// int next_h_val = my_heuristic[next_location];
			int next_h_val = max(get_heuristic(stage, next_location), holding_time - next_timestep);
			// if (next_g_val + next_h_val > constraint_table.length_max || next_g_val + next_h_val > f_ub[stage])
			// 	continue;
      // TODO use CAT
			int next_internal_conflicts = curr->num_of_conflicts;
      // constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

			// generate (maybe temporary) node
			auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val,
                                curr, next_timestep, stage, next_internal_conflicts, false);

      next->timestamps  = timestamps;
      next->secondary_keys.push_back(-next_g_val);

      next->dist_to_next = my_heuristic[stage][next_location];

			if (next->stage == goal_location.size() - 1 && next_location == goal_location.back() && curr->location == goal_location.back()){
				next->wait_at_goal = true;
      }

			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				pushNode(next);
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
				(existing_next->getFVal() == next->getFVal() &&
         LLNode::secondary_compare_node_not_random()(existing_next, next)
				 // existing_next->num_of_conflicts > next->num_of_conflicts
         )) // or it remains the same but there's fewer conflicts
			{
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					existing_next->copy(*next);
					pushNode(existing_next);
				}
				else
				{
					bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
					bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
					bool update_open = false;
					if ((next_g_val + next_h_val) <= lower_bound)
					{  // if the new f-val qualify to be in FOCAL
						if (existing_next->getFVal() > lower_bound)
							add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
						else
							update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
					}
					if (existing_next->getFVal() > next_g_val + next_h_val)
						update_open = true;

					existing_next->copy(*next);  // update existing node

					if (update_open)
						open_list.increase(existing_next->open_handle);  // increase because f-val improved
					if (add_to_focal)
						existing_next->focal_handle = focal_list.push(existing_next);
					if (update_in_focal)
						focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down			
				}
			}
			delete next;  // not needed anymore -- we already generated it before
		}  // end for loop that generates successors
	}  // end while loop

	releaseNodes();
	return path;
}



Path MultiLabelSpaceTimeAStar::findPath(ConstraintTable& constraint_table, const pair<int, int> start_state, const pair<int, int> goal_state)
{
  /*
	// generate start and add it to the OPEN & FOCAL list
	Path path;
	auto start = new AStarNode(start_state.first,  // location
		0,  // g val 
		compute_heuristic(start_state.first, goal_state.first),  // h val
		nullptr,  // parent
		start_state.second,  // timestep
		0, false);
	if (start->timestep + start->h_val > goal_state.second)
		return path;
	num_generated++;
	start->focal_handle = focal_list.push(start);
	allNodes_table.insert(start);
	// min_f_val = (int)start->getFVal();

	while (!focal_list.empty())
	{
		auto* curr = focal_list.top(); 
		focal_list.pop();
		curr->in_openlist = false;

		// check if the popped node is a goal
		if (curr->location == goal_state.first && // arrive at the goal location
			curr->timestep == goal_state.second) // at the corresponding timestep
		{
			updatePath(curr, path);
			break;
		}

		num_expanded++;
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;

			if (constraint_table.constrained(next_location, next_timestep) ||
				constraint_table.constrained(curr->location, next_location, next_timestep))
				continue;

			// compute cost to next_id via curr node
			int next_g_val = curr->g_val + 1;
			int next_h_val = compute_heuristic(next_location, goal_state.first);
			if (next_timestep + next_h_val > goal_state.second)
				continue;
			int next_internal_conflicts = curr->num_of_conflicts +
				constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

			// generate (maybe temporary) node
			auto next = new AStarNode(next_location, next_g_val, next_h_val,
				curr, next_timestep, next_internal_conflicts, false);
			
			// try to retrieve it from the hash table
			auto it = allNodes_table.find(next);
			if (it == allNodes_table.end())
			{
				num_generated++;
				next->focal_handle = focal_list.push(next);
				next->in_openlist = true;
				allNodes_table.insert(next);
				continue;
			}
			// update existing node's if needed (only in the open_list)

			auto existing_next = *it;
			if (existing_next->num_of_conflicts > next->num_of_conflicts) // if there's fewer conflicts
			{
				existing_next->copy(*next);	// update existing node
				if (!existing_next->in_openlist) // if its in the closed list (reopen)
				{
					next->focal_handle = focal_list.push(existing_next);
					existing_next->in_openlist = true;
				}
				else
				{
					focal_list.update(existing_next->focal_handle);		
				}
			}
			delete next;  // not needed anymore -- we already generated it before
		}  // end for loop that generates successors
	}  // end while loop

	releaseNodes();
	return path;
  */
}


int MultiLabelSpaceTimeAStar::getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound)
{
	int length = MAX_TIMESTEP;
	if (constraint_table.length_min >= MAX_TIMESTEP || constraint_table.length_min > constraint_table.length_max ||  // the agent cannot reach
																													 // its goal location
		constraint_table.constrained(start, 0)) // the agent cannot stay at its start location
	{
		return length;
	}
	auto root = new MultiLabelAStarNode(start, 0, compute_heuristic(start, end), nullptr, 0, 0);
	root->open_handle = open_list.push(root);  // add root to heap
	allNodes_table.insert(root);       // add root to hash_table (nodes)
	MultiLabelAStarNode* curr = nullptr;
	while (!open_list.empty())
	{
		curr = open_list.top(); open_list.pop();
		if (curr->location == end)
		{
			length = curr->g_val;
			break;
		}
		list<int> next_locations = instance.getNeighbors(curr->location);
		next_locations.emplace_back(curr->location);
		for (int next_location : next_locations)
		{
			int next_timestep = curr->timestep + 1;
			int next_g_val = curr->g_val + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (curr->location == next_location)
				{
					continue;
				}
				next_timestep--;
			}
			if (!constraint_table.constrained(next_location, next_timestep) &&
				!constraint_table.constrained(curr->location, next_location, next_timestep))
			{  // if that grid is not blocked

        // setting the stage
        auto stage = curr->stage;
        if (next_location == goal_location[stage] && stage < goal_location.size() - 1){
          stage += 1;
        }

				int next_h_val = compute_heuristic(next_location, end);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;

				auto next = new MultiLabelAStarNode(next_location, next_g_val, next_h_val, nullptr, next_timestep, stage);
				auto it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = open_list.push(next);
					allNodes_table.insert(next);
				}
				else
				{  // update existing node's g_val if needed (only in the heap)
					delete next;  // not needed anymore -- we already generated it before
					auto existing_next = *it;
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						open_list.increase(existing_next->open_handle);
					}
				}
			}
		}
	}
	releaseNodes();
	return length;
}

inline MultiLabelAStarNode* MultiLabelSpaceTimeAStar::popNode()
{
	auto node = focal_list.top(); focal_list.pop();
	open_list.erase(node->open_handle);
	node->in_openlist = false;
	num_expanded++;
	return node;
}


inline void MultiLabelSpaceTimeAStar::pushNode(MultiLabelAStarNode* node)
{
	node->open_handle = open_list.push(node);
	node->in_openlist = true;
	num_generated++;
	if (node->getFVal() <= lower_bound)
		node->focal_handle = focal_list.push(node);		
}


void MultiLabelSpaceTimeAStar::updateFocalList()
{
	auto open_head = open_list.top();
	if (open_head->getFVal() > min_f_val)
	{
		int new_min_f_val = (int) open_head->getFVal();
		int new_lower_bound = max(lower_bound, new_min_f_val);
		for (auto n : open_list)
		{
			if (n->getFVal() > lower_bound && n->getFVal() <= new_lower_bound)
				n->focal_handle = focal_list.push(n);
		}
		min_f_val = new_min_f_val;
		lower_bound = new_lower_bound;
	}
}


void MultiLabelSpaceTimeAStar::releaseNodes()
{
	open_list.clear();
	focal_list.clear();
	for (auto node: allNodes_table)
		delete node;
	allNodes_table.clear();
}

