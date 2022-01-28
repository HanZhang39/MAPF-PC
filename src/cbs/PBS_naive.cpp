#include "PBS.h"
#include "SpaceTimeAStar.h"
#include <stack>

void PBS_naive::get_adj_list(CBSNode* node, vector<vector<int>>& adj_list){
  adj_list.clear();
  adj_list.resize(num_of_agents);
  while (node!= nullptr){
    for (auto con: node->constraints){
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      adj_list[a].push_back(x);
    }

    node = node->parent;
  }
}

void PBS_naive::get_adj_list(CBSNode* node, vector<vector<int>>& adj_list, vector<vector<int>>& adj_list_r){
  adj_list.clear();
  adj_list.resize(num_of_agents);
  adj_list_r.clear();
  adj_list_r.resize(num_of_agents);
  while (node!= nullptr){
    for (auto con: node->constraints){
      int a, x, y, t;
      constraint_type type;
      tie(a, x, y, t, type) = con;
      adj_list[a].push_back(x);
      adj_list_r[x].push_back(a);
    }

    node = node->parent;
  }
}


bool PBS_naive::topological_sort(vector<vector<int>>& adj_list, vector<int>& planning_order){
  planning_order.clear();
  vector<bool> closed(num_of_agents, false);
  vector<bool> expanded(num_of_agents, false);
  for (int i = 0 ; i < num_of_agents; i++){
    if (closed[i]){
      continue;
    }
    std::stack<int> dfs_stack;
    dfs_stack.push(i);
    while(!dfs_stack.empty()){
      auto task = dfs_stack.top();
      dfs_stack.pop();
      if (closed[task]){
        continue;
      }
      if (expanded[task]){
        closed[task] = true;
        planning_order.push_back(task);
      } else {
        expanded[task] = true;
        dfs_stack.push(task);
        for (auto ch: adj_list[task]){
          if (closed[ch]){
            continue;
          }
          if (expanded[ch]){
            cout << "detect cycle";
            return false;
          }
          dfs_stack.push(ch);
        }
      }
    }
  }
  std::reverse(planning_order.begin(), planning_order.end());

  // toposort checker
  unordered_set<int> high;
  for (auto i: planning_order){
    for (auto j: adj_list[i]){
      if (high.find(j) != high.end()){
        cout << "REVERSE EDGES!" << endl;
        assert(false);
      }
    }
    high.insert(i);
  }

  assert(planning_order.size() == num_of_agents);
  return true;
}

bool PBS_naive::solve(double time_limit, int cost_lowerbound, int cost_upperbound)
{
	this->min_f_val = cost_lowerbound;
	this->cost_upperbound = cost_upperbound;
	this->time_limit = time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": ";
	}
	// set timer
	start = clock();


	generateRoot();
  if (dummy_start->conflicts.size() == 0){
    solution_found = true;
    solution_cost = dummy_start->g_val;
    goal_node = dummy_start;
  }

  std::stack<CBSNode*> dfs_stack;
  dfs_stack.push(dummy_start);

	while (!dfs_stack.empty() && !solution_found)
	{
		// updateFocalList();
		// if (min_f_val >= cost_upperbound)
		// {
		// 	solution_cost = (int) min_f_val;
		// 	solution_found = false;
		// 	break;
		// }
		runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
		if (runtime > time_limit || num_HL_expanded > node_limit)
		{  // time/node out
			solution_cost = -1;
			solution_found = false;
			break;
		}
		CBSNode* curr = dfs_stack.top();
		dfs_stack.pop();
		// open_list.erase(curr->open_handle);
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr);

		if (screen > 1)
			cout << endl << "Pop " << *curr << endl;

		//Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;
    CBSNode* child[2] = { new CBSNode(), new CBSNode() };

    curr->conflict = chooseConflict(*curr);
    
    child[0]->constraints = curr->conflict->constraint1;
    child[1]->constraints = curr->conflict->constraint2;

    if (screen > 1)
      cout << "	Expand " << *curr << endl <<
        "	on " << *(curr->conflict) << endl;

    bool solved[2] = { false, false };
    vector<Path*> copy(paths);

    for (int i = 0; i < 2; i++)
			{
				if (i > 0)
					paths = copy;
				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
          {
            cout << "gen child " << i << " failed" << endl;
            delete child[i];
            continue;
          }
				if (child[i]->conflicts.size() == 0) //no conflicts
          {// found a solution (and finish the while look)
            solution_found = true;
            solution_cost = child[i]->g_val;
            goal_node = child[i];
            updatePaths(child[i]);
            break;
          }
			}
    // if (solved[0] && solved[1]){
    //   if (child[0]->g_val < child[1]->g_val){
    //     std::swap(child[0], child[1]);
    //   }
    // }

    for (int i = 0; i < 2; i++)
      {
        if (solved[i])
          {
            // pushNode(child[i]);
            num_HL_generated++;
            child[i]->time_generated = num_HL_generated;
            dfs_stack.push(child[i]);
            if (screen > 1)
              {
                cout << "		Generate " << *child[i] << endl;
              }
          }
      }
		curr->clear();
	}  // end of while loop


	runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
	if (solution_found && !validateSolution())
	{
		cout << "Solution invalid!!!" << endl;
		printPaths();
		exit(-1);
	}

	if (screen > 0){ // 1 or 2
		printResults();
    if (solution_found){
      printPaths();
    }
  }
	return solution_found;
}

void PBS_naive::build_ct(ConstraintTable& ct, int agent, vector<vector<int>> adj_list_r){

  int num_of_stops = search_engines[agent]->goal_location.size();
  ct.leq_goal_time.resize(num_of_stops, INT_MAX);
  ct.g_goal_time.resize(num_of_stops, -1);

  ct.num_col = search_engines[0]->instance.num_of_cols; 
  ct.map_size = search_engines[0]->instance.map_size;
  ct.goal_location = search_engines[agent]->goal_location.back();

  auto high_prio_agents = reachable_set(agent, adj_list_r);
  high_prio_agents.erase(agent);

  //TODO remove later
  cout << "hard cons: ";
  for (int i = 0; i < num_of_agents; i++){
    if (high_prio_agents.find(i) != high_prio_agents.end()){
      // int agent, task;
      ct.addPath(*paths[i], true);

      cout <<  i  << ", ";



      for (auto cons: search_engines[0]->instance.temporal_cons[agent * num_of_agents + i]){
        auto from_landmark = cons.first;
        auto to_landmark = cons.second;
        // if (ct.leq_goal_time[from_landmark] == -1){
        //   ct.leq_goal_time[from_landmark] = paths[i] ->timestamps[to_landmark] - 1;
        // }else{
          ct.leq_goal_time[from_landmark] = min(ct.leq_goal_time[from_landmark], paths[i] ->timestamps[to_landmark] - 1);
          // }
        // if (paths[agent]->timestamps[from_landmark] >= paths[i]->timestamps[to_landmark]){
      }
      for (auto cons: search_engines[0]->instance.temporal_cons[i * num_of_agents + agent]){
        auto from_landmark = cons.first;
        auto to_landmark = cons.second;
        ct.g_goal_time[to_landmark] = max(ct.g_goal_time[to_landmark], paths[i] ->timestamps[from_landmark]);
        ct.length_min = max(ct.length_min, paths[i] ->timestamps[from_landmark] + 1);
        // if (paths[i]->timestamps[from_landmark] >= paths[agent]->timestamps[to_landmark]){
      }
    }
  }
  cout << endl;
  // cout << "soft cons: ";
  // for (int i = 0; i < num_of_tasks; i++){
  //   if (high_prio_agents.find(i) == high_prio_agents.end() && paths[i] != nullptr && !paths[i]->empty()){
  //     auto task = id2task[i];
  //     cout << "(" << task.first << ", " << task.second << ") ";
  //   }
  // }
  // cout << endl;

  // // temporal cons
  // for (auto precedent: temporal_adj_list_r[task_id]){
  //   assert(!paths[precedent]->empty());
  //   ct.length_min = max(ct.length_min, paths[precedent]->end_time() + 1);
  // }
  // ct.latest_timestep = max(ct.latest_timestep, ct.length_min);

}

bool PBS_naive::generateChild(CBSNode* node, CBSNode* parent){
	clock_t t1 = clock();
  node->parent = parent;
  node->makespan = parent->makespan;
  vector<Path*> copy(paths);
  vector<vector<int>> adj_list, adj_list_r;
  get_adj_list(node, adj_list, adj_list_r);

  // remove paths that are affected;
  auto affected_agents= reachable_set(std::get<1>(node->constraints.front()), adj_list);
  for (auto agent:affected_agents){
    node->paths.emplace_back(agent, Path());
    paths[agent] = &node->paths.back().second;
  }

  vector<int> planning_order;

  bool has_cycle = !topological_sort(adj_list, planning_order);
  if (has_cycle){
    cout << "cycle in priority graph" << endl;
    return false;
  }

  // TOOD remove this code block
  cout << "after sorting" << endl;
  for (auto agent: planning_order){
    cout << "(" << agent << ")";
    if (!paths[agent]->empty()){
      cout << "+";
    }
    if (affected_agents.find(agent) != affected_agents.end()){
      cout << "*";
    }
    cout << " ";
  }
  cout << endl;

  node->is_solution = true;

  list<int> excluded_agents;

  // replanning
  for (auto i : planning_order)
    {
      if (paths[i]->empty()){
        cout << "replanning " << i << ": " << paths[i] << endl;

        cout << "plan for " << i << endl;
        ConstraintTable ct;
        build_ct(ct, i, adj_list_r);

        bool reuse_old_path = false;
        /*
        if (copy[i]->end_time() >= ct.length_min){
          reuse_old_path = true;
          for (int j = 0; j + 1 < copy[i]->size(); j++){
            int actual_t = j + copy[i]->begin_time;
            int loc = copy[i]->at(j).location;
            int next_loc = copy[i]->at(j + 1).location;
            if (ct.constrained(loc, actual_t)){
              reuse_old_path = false;
              break;
            }
            if (ct.constrained(loc, next_loc, actual_t + 1)){
              reuse_old_path = false;
              break;
            }
          }
          if (is_task_a_final_one(i)){
            if (copy[i]->end_time() < ct.getHoldingTime()){
              reuse_old_path = false;
            }
          } else {
            if (ct.constrained(ct.goal_location, copy[i]->end_time())){
              reuse_old_path = false;
            }
          }
        }
        */

        if (reuse_old_path){
          cout << "reuse old path" << endl;
          *paths[i] = *copy[i];
        }else{
          // *paths[i] = search_engines[agent]->findPathSegment(ct, start_time, task, 0);
          int lower_bound = (int) copy[i]->size() - 1;
          *paths[i] = search_engines[i] ->findPath(*node, ct, paths, i, lower_bound);
          excluded_agents.push_back(i);
          node->makespan = max(node->makespan, paths[i]->size() - 1);
        }
        if (paths[i]->empty())
          {
            cout << "No path exists for agent " << i << endl;
            return false;
          }
      }

      auto high_prio_agents = reachable_set(i, adj_list_r);
      high_prio_agents.erase(i);
      for (auto j: high_prio_agents){
        bool is_conf = findOneConflict(i, j);
        if (is_conf){
          cout << "between " << i << " and " << j << endl;
          cout << paths[i] << " " << paths[j] << endl;
        }
        assert(! is_conf);
      }

      bool conflict_found = false;

      for (auto j : planning_order){
        if (i == j){
          break;
        }
        if (high_prio_agents.find(j) == high_prio_agents.end()){
          if (findOneConflict(i, j)){
            cout << "Conflict between " << i <<" and " << j << endl;
            shared_ptr<Conflict> conflict(new Conflict());
            conflict->priorityConflict(i, j);
            node->conflicts.push_back(conflict);
            // node->conflict = conflict;
            // node->is_solution = false;
            // conflict_found = true;
            // break;
          }
        }
      }

      num_LL_expanded += search_engines[i]->num_expanded;
      num_LL_generated += search_engines[i]->num_generated;

      if (conflict_found){
        break;
      }
    }

  // copy conflicts from parent
    copyConflicts(node->parent->conflicts, node->conflicts, excluded_agents);

  // compute g
  node->g_val = 0;
  for (int i = 0; i < num_of_agents; i++ ){
    node->g_val += paths[i]->size();
  }

  runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
  return true;
}

bool PBS_naive::generateRoot()
{
  dummy_start = new CBSNode();
  dummy_start->g_val = 0;
  paths.resize(num_of_agents, nullptr);
  vector<vector<int>> adj_list, adj_list_r;
  get_adj_list(dummy_start, adj_list, adj_list_r);
  // get_adj_list(dummy_start, temporal_adj_list, temporal_adj_list_r);

  // TOOD remove this code block
  // cout << "adj list:" << endl;
  // for (int i = 0; i < adj_list.size(); i++){
  //   for (auto j: adj_list[i]){
  //     cout << i << " -> " << j << endl;
  //   }
  // }

  // mdd_helper.init(num_of_agents);
  // heuristic_helper->init();

  vector<int> planning_order;

  topological_sort(adj_list, planning_order);

  // initialize paths_found_initially
  paths_found_initially.resize(num_of_agents, Path());

  dummy_start -> is_solution = true;
  for (auto i : planning_order)
    {
      //CAT cat(dummy_start->makespan + 1);  // initialized to false
      //updateReservationTable(cat, i, *dummy_start);
      int agent = i;
      int start_time = 0;

      cout << "plan for " << agent << endl;
      ConstraintTable ct;
      build_ct(ct, i, adj_list_r);

      // TODO lowerbound ????
      paths_found_initially[i] = search_engines[i] ->findPath(*dummy_start, ct, paths, i, 0);
      if (paths_found_initially[i].empty())
        {
          cout << "No path exists for agent " << agent << endl;
          return false;
        }
      paths[i] = &paths_found_initially[i];

      bool conflict_found = false;

      for (auto j : planning_order){
        if (i == j){
          break;
        }
        if (findOneConflict(i, j)){
          cout << "Conflict between " << i <<" and " << j << endl;
          shared_ptr<Conflict> conflict(new Conflict());
          conflict->priorityConflict(i, j);
          // dummy_start->conflict = conflict;
          // dummy_start->is_solution = false;
          // conflict_found = true;
          dummy_start->conflicts.push_back(conflict);
        }
      }

      dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
      // dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
      num_LL_expanded += search_engines[agent]->num_expanded;
      num_LL_generated += search_engines[agent]->num_generated;

    }

  for (int i = 0; i < num_of_agents; i++){
    paths[i] = &paths_found_initially[i];
  }

  // generate dummy start and update data structures
  dummy_start->h_val = 0;
  dummy_start->depth = 0;
  // dummy_start->open_handle = open_list.push(dummy_start);
  // dummy_start->focal_handle = focal_list.push(dummy_start);

  num_HL_generated++;
  dummy_start->time_generated = num_HL_generated;
  allNodes_table.push_back(dummy_start);
  // we don't need to use findConflicts in PBS?
  // findConflicts(*dummy_start);

  // We didn't compute the node-selection tie-breaking value for the root node
  // since it does not need it.
  // min_f_val = max(min_f_val, (double) dummy_start->g_val);
  // focal_list_threshold = min_f_val * focal_w;

  if (screen >= 2) // print start and goals
  {
    // printPaths();
  }

  return true;
}

PBS_naive::~PBS_naive(){
	releaseNodes();
	mdd_helper.clear();
}

string PBS_naive::getSolverName() const
{
  return "PBS Naive";
}

shared_ptr<Conflict> PBS_naive::chooseConflict(const CBSNode& node) const {
  auto start = node.conflicts.begin();
  std::advance(start, std::rand() % node.conflicts.size() );
  return *start;
}


bool PBS_naive::findOneConflict(int a1, int a2){

  
  for (auto cons: search_engines[0]->instance.temporal_cons[a1 * num_of_agents + a2]){
    auto from_landmark = cons.first;
    auto to_landmark = cons.second;
    if (paths[a1]->timestamps[from_landmark] >= paths[a2]->timestamps[to_landmark]){
      // cout << "Temporal conflict between " << a1  << "(" << from_landmark<< ")" << " and " << a2 << "(" << to_landmark<< ")" << endl;
      shared_ptr<Conflict> conflict(new Conflict());
      conflict->temporalConflict(a1, a2, from_landmark, to_landmark, paths[a1]->timestamps[from_landmark], paths[a2]->timestamps[to_landmark]);
      return true;
    }
  }
  for (auto cons: search_engines[0]->instance.temporal_cons[a2 * num_of_agents + a1]){
    auto from_landmark = cons.first;
    auto to_landmark = cons.second;
    if (paths[a2]->timestamps[from_landmark] >= paths[a1]->timestamps[to_landmark]){
      // cout << "Temporal conflict between " << a2  << "(" << from_landmark<< ")" << " and " << a1 << "(" << to_landmark<< ")" << endl;
      shared_ptr<Conflict> conflict(new Conflict());
      conflict->temporalConflict(a2, a1, from_landmark, to_landmark, paths[a2]->timestamps[from_landmark], paths[a1]->timestamps[to_landmark]);
      return true;
    }
  }

  assert(paths[a1] != nullptr && !paths[a1]->empty());
  assert(paths[a2] != nullptr && !paths[a2]->empty());
  if (paths[a1]->end_time() > paths[a2]->end_time()){
    std::swap(a1, a2);
  }
  for (int t = max(paths[a1]->begin_time, paths[a2]->begin_time) + 1; t < paths[a1]->end_time() + 1; t++){
    int i1 = t - paths[a1]->begin_time;
    int i2 = t - paths[a2]->begin_time;
    if (paths[a1]->at(i1).location == paths[a2]->at(i2).location){
      cout << "vertex conf! " << i1 << "," << i2 << endl;
      return true;
    }
    if (paths[a1]->at(i1 - 1).location == paths[a2]->at(i2).location && paths[a1]->at(i1).location == paths[a2]->at(i2 - 1).location){
      cout << "edge conf! " << i1 << ", " << i2  << endl;
      return true;
    }
  }

  // goal conf
  int goal_loc_1 = paths[a1]->back().location;
  for (int t = max(paths[a1]->end_time(), paths[a2]->begin_time + 1); t < paths[a2]->end_time(); t++){
    int i2 = t - paths[a2]->begin_time;
    if (goal_loc_1 == paths[a2]->at(i2).location){
      cout << "goal vertex conf!" << endl;
      return true;
    }
  }

  return false;
}



PBS_naive::PBS_naive(const Instance& instance, int screen):
CBS(instance, false, heuristics_type::ZERO, screen)
{
  this->screen = screen;
  this->focal_w = 1;
  this->num_of_agents = instance.getDefaultNumberOfAgents();
  // mdd_helper(initial_constraints, search_engines),
	clock_t t = clock();

	search_engines.resize(num_of_agents);


	runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}

