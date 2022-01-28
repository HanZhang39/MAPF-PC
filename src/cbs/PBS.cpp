#include "PBS.h"
#include "SpaceTimeAStar.h"
#include <stack>


void PBS::printResults() const
{
  if (solution_cost >= 0) // solved
    cout << "Solved,";
  else if (solution_cost == -1) // time_out
    cout << "Timeout,";
  else if (solution_cost == -2) // no solution
    cout << "No solutions,";
  else if (solution_cost == -3) // nodes out
    cout << "Nodesout,";

  cout << solution_cost << "," << runtime << "," <<
    num_HL_expanded << "," << num_LL_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
    min_f_val << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
    endl;
}

void PBS::printPaths() const
{
  const Instance* instance = & search_engines[0]->instance;
  for (int i = 0; i < num_of_agents; i++)
    {
      cout << "Agent " << i << " (cost =  " <<
        paths[i]->size() - 1 << "): ";

      for (int t = 0; t < paths[i]->size(); t++){
        cout << "(" << instance->getRowCoordinate(paths[i]->at(t).location) << ", " << instance->getColCoordinate(paths[i]->at(t).location) << ")@" << t;
        if (paths[i]->at(t).is_goal){
          cout << "*";
        }
        cout << "->";
      }
      cout << endl;
      for (int j = 0; j < paths[i]->timestamps.size(); j++){
        cout << "(" << instance->getRowCoordinate(search_engines[i]->goal_location[j]) << ", " << instance->getColCoordinate(search_engines[i]->goal_location[j]) << ")@" << paths[i]->timestamps[j];
        cout << "->";
      }
      cout << endl;
    }
}

inline bool PBS::is_task_a_final_one(int task){
  int agent, i;
  tie(agent, i) = id2task[task];
  return i == search_engines[agent]->goal_location.size() - 1;
}

inline void PBS::updatePaths(CBSNode* curr)
{
  for (int i = 0; i < num_of_tasks; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_tasks, false);  // initialized for false

	while (curr != nullptr)
    {
      for (auto it = curr->paths.begin(); it != curr->paths.end(); ++it)
        {
          if (!updated[it->first])
            {
              paths[it->first] = &(it->second);
              updated[it->first] = true;
            }
        }
      curr = curr->parent;
    }
}


PBS::PBS(const Instance& instance, int screen):
CBS(instance, false, heuristics_type::ZERO, screen)
{
  this->screen = screen;
  this->focal_w = 1;
  this->num_of_agents = instance.getDefaultNumberOfAgents();
  // mdd_helper(initial_constraints, search_engines),
	clock_t t = clock();

	search_engines.resize(num_of_agents);
  idbase.resize(num_of_agents, 0);

	for (int i = 0; i < num_of_agents; i++)
	{
    search_engines[i] = new MultiLabelSpaceTimeAStar(instance, i);
    for (int j = 0; j < search_engines[i]->goal_location.size(); j++){
      id2task.push_back({i, j});
    }
    if (i != 0){
      idbase[i] = idbase[i - 1] + search_engines[i - 1]->goal_location.size();
    }
	}

  // initialize priorities
	for (int i = 0; i < num_of_agents; i++)
    {
      for (int j = 1; j < search_engines[i]->goal_location.size(); j++){
        initial_priorities.push_back({task2id({i, j - 1}), task2id({i, j}), -1, -1, constraint_type::GPRIORITY});
      }
    }
  for (int i = 0; i < instance.temporal_cons.size(); i++){
    if (instance.temporal_cons[i].empty()){
      continue;
    }
    int from_agent = i / num_of_agents;
    int to_agent = i % num_of_agents;
    for (auto landmarks: instance.temporal_cons[i]){
      int from_landmark = landmarks.first;
      int to_landmark = landmarks.second;
      cout << "add " << from_agent << "(" << from_landmark << ") " << to_agent << "(" << to_landmark << ")" << endl; 
      initial_priorities.push_back({task2id({from_agent, from_landmark}), task2id({to_agent, to_landmark}), -1, -1, constraint_type::GPRIORITY});
    }
  }

  num_of_tasks = id2task.size();

	runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}



void PBS::get_adj_list(CBSNode* node, vector<vector<int>>& adj_list){
  adj_list.clear();
  adj_list.resize(num_of_tasks);
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

void PBS::get_adj_list(CBSNode* node, vector<vector<int>>& adj_list, vector<vector<int>>& adj_list_r){
  adj_list.clear();
  adj_list.resize(num_of_tasks);
  adj_list_r.clear();
  adj_list_r.resize(num_of_tasks);
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


bool PBS::topological_sort(vector<vector<int>>& adj_list, vector<int>& planning_order){
  planning_order.clear();
  vector<bool> closed(num_of_tasks, false);
  vector<bool> expanded(num_of_tasks, false);
  for (int i = 0 ; i < num_of_tasks; i++){
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

  assert(planning_order.size() == num_of_tasks);
  return true;
}

unordered_set<int> reachable_set(int source, vector<vector<int>> adj_list){
  unordered_set<int> res;
  std::stack<int> q({source});
  while(!q.empty()){
    int curr = q.top();
    q.pop();
    if (res.find(curr) != res.end()){
      continue;
    }
    res.insert(curr);
    for(auto ch: adj_list[curr]){
      if (res.find(ch) == res.end()){
        q.push(ch);
      }
    }
  }
  return res;
}

void PBS::build_ct(ConstraintTable& ct, int task_id, vector<vector<int>> adj_list_r){

  int agent, task;
  tie(agent, task) = id2task[task_id];
  ct.goal_location = search_engines[agent]->goal_location[task];

  auto high_prio_agents = reachable_set(task_id, adj_list_r);
  high_prio_agents.erase(task_id);

  //TODO remove later
  cout << "Higher-priority tasks: ";
  for (int i = 0; i < num_of_tasks; i++){
    if (high_prio_agents.find(i) != high_prio_agents.end()){
      // int agent, task;
      tie(agent, task) = id2task[i];
      bool wait_at_goal = task == search_engines[agent]->goal_location.size() - 1;
      ct.addPath(*paths[i], wait_at_goal);

      cout << "(" << agent << ", " << task << ") ";
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

  // temporal cons
  for (auto precedent: temporal_adj_list_r[task_id]){
    assert(!paths[precedent]->empty());
    ct.length_min = max(ct.length_min, paths[precedent]->end_time() + 1);
  }
  ct.latest_timestep = max(ct.latest_timestep, ct.length_min);

}


bool PBS::findOneConflict(int task1, int task2){
  assert(paths[task1] != nullptr && !paths[task1]->empty());
  assert(paths[task2] != nullptr && !paths[task2]->empty());
  if (paths[task1]->end_time() > paths[task2]->end_time()){
    std::swap(task1, task2);
  }
  for (int t = max(paths[task1]->begin_time, paths[task2]->begin_time) + 1; t < paths[task1]->end_time() + 1; t++){
    int i1 = t - paths[task1]->begin_time;
    int i2 = t - paths[task2]->begin_time;
    if (paths[task1]->at(i1).location == paths[task2]->at(i2).location){
      cout << "vertex conf! " << i1 << "," << i2 << endl;
      return true;
    }
    if (paths[task1]->at(i1 - 1).location == paths[task2]->at(i2).location && paths[task1]->at(i1).location == paths[task2]->at(i2 - 1).location){
      cout << "edge conf! " << i1 << ", " << i2  << endl;
      return true;
    }
  }
  if (is_task_a_final_one(task1)){
    int goal_loc_1 = paths[task1]->back().location;
    for (int t = max(paths[task1]->end_time(), paths[task2]->begin_time + 1); t < paths[task2]->end_time(); t++){
      int i2 = t - paths[task2]->begin_time;
      if (goal_loc_1 == paths[task2]->at(i2).location){
        cout << "goal vertex conf!" << endl;
        return true;
      }
    }
  }

  return false;
}

bool PBS::generateChild(CBSNode* node, CBSNode* parent){
	clock_t t1 = clock();
  node->parent = parent;
  vector<Path*> copy(paths);
  vector<vector<int>> adj_list, adj_list_r;
  get_adj_list(node, adj_list, adj_list_r);

  // remove paths that are affected;
  auto affected_tasks = reachable_set(std::get<1>(node->constraints.front()), adj_list);
  for (auto task:affected_tasks){
    node->paths.emplace_back(task, Path());
    paths[task] = &node->paths.back().second;
  }

  vector<int> planning_order;

  bool has_cycle = !topological_sort(adj_list, planning_order);
  if (has_cycle){
    cout << "cycle in priority graph" << endl;
    return false;
  }

  // cout << "after sorting" << endl;
  // for (auto task_id: planning_order){
  //   auto task = id2task[task_id];
  //   cout << "(" << task.first << ", " << task.second << ")";
  //   if (!paths[task_id]->empty()){
  //     cout << "+";
  //   }
  //   if (affected_tasks.find(task_id) != affected_tasks.end()){
  //     cout << "*";
  //   }
  //   cout << " ";
  // }
  // cout << endl;

  node->is_solution = true;

  // replanning
  for (auto i : planning_order)
    {
      int agent, task;
      ConstraintTable ct;
      tie(agent, task) = id2task[i];
      if (paths[i]->empty()){
        cout << "replanning " << i << endl;
        int start_time = 0;

        if (task != 0){
          assert(!paths[task2id({agent, task - 1})]->empty());
          start_time = paths[task2id({agent, task - 1})]->end_time();
        }

        cout << "plan for " << agent << "(" << task << ")"<< endl;
        build_ct(ct, i, adj_list_r);

        bool reuse_old_path = false;
        if (copy[i]->begin_time == start_time && copy[i]->end_time() >= ct.length_min){
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

        if (reuse_old_path){
          cout << "reuse old path" << endl;
          *paths[i] = *copy[i];
        }else{
          *paths[i] = search_engines[agent]->findPathSegment(ct, start_time, task, 0);
        }
        if (paths[i]->empty())
          {
            cout << "No path exists for agent " << agent << "(" << task << ")" << endl;
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
            node->conflict = conflict;
            node->is_solution = false;
            conflict_found = true;
            break;
          }
        }
      }

      num_LL_expanded += search_engines[agent]->num_expanded;
      num_LL_generated += search_engines[agent]->num_generated;

      if (conflict_found){
        break;
      }
    }

  node->g_val = 0;
  for (int i = 0; i < num_of_agents; i++ ){
    int g_i = search_engines[i]->heuristic_landmark[0];
    for (int j = 0; j < search_engines[i]->goal_location.size(); j ++ ){
      int task_id = task2id({i, j});
      if (paths[task_id]->empty()) {
        break;
      }
      g_i = paths[task_id]->end_time();
      if (j + 1 < search_engines[i]->goal_location.size()){
        g_i += search_engines[i]->heuristic_landmark[j + 1];
      }
    }
    node->g_val += g_i;
  }
  // compute g


  runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
  return true;
}

bool PBS::generateRoot()
{
  dummy_start = new CBSNode();
  dummy_start->g_val = 0;
  paths.resize(num_of_tasks, nullptr);
  for (auto con:initial_priorities){
    dummy_start->constraints.push_back(con);
  }
  vector<vector<int>> adj_list, adj_list_r;
  get_adj_list(dummy_start, adj_list, adj_list_r);
  get_adj_list(dummy_start, temporal_adj_list, temporal_adj_list_r);

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

  // cout << "after sorting" << endl;
  // for (auto task_id: planning_order){
  //   auto task = id2task[task_id];
  //   cout << "(" << task.first << ", " << task.second << ") ";
  // }
  // cout << endl;


  // initialize paths_found_initially
  paths_found_initially.resize(num_of_tasks, Path());

  dummy_start -> is_solution = true;
  for (auto i : planning_order)
    {
      //CAT cat(dummy_start->makespan + 1);  // initialized to false
      //updateReservationTable(cat, i, *dummy_start);
      int agent, task;
      tie(agent, task) = id2task[i];
      int start_time = 0;
      if (task != 0){
        assert(!paths_found_initially[task2id({agent, task - 1})].empty());
        start_time = paths_found_initially[task2id({agent, task - 1})].end_time();
      }

      cout << "plan for " << agent << "(" << task << ")"<< endl;
      ConstraintTable ct;
      build_ct(ct, i, adj_list_r);

      paths_found_initially[i] = search_engines[agent]->findPathSegment(ct, start_time, task, 0);
      if (paths_found_initially[i].empty())
        {
          cout << "No path exists for agent " << agent << "(" << task << ")" << endl;
          return false;
        }
      paths[i] = &paths_found_initially[i];

      auto high_prio_agents = reachable_set(i, adj_list_r);
      high_prio_agents.erase(i);
      for (auto j: high_prio_agents){
        assert(! findOneConflict(i, j));
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
            dummy_start->conflict = conflict;
            dummy_start->is_solution = false;
            conflict_found = true;
          }
        }
      }

      // dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
      // dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
      num_LL_expanded += search_engines[agent]->num_expanded;
      num_LL_generated += search_engines[agent]->num_generated;

      if (conflict_found){
        break;
      }
    }

  for (int i = 0; i < num_of_tasks; i++){
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

  dummy_start->g_val += 0;
  for (int i = 0; i < num_of_agents; i++ ){
    int g_i = search_engines[i]->heuristic_landmark[0];
    for (int j = 0; j < search_engines[i]->goal_location.size(); j ++ ){
      int task_id = task2id({i, j});
      if (paths[task_id]->empty()) {
        break;
      }
      g_i = paths[task_id]->end_time();
      if (j + 1 < search_engines[i]->goal_location.size()){
        g_i += search_engines[i]->heuristic_landmark[j + 1];
      }
    }
    dummy_start->g_val += g_i;
  }
  // compute g
  return true;
}


bool PBS::solve(double time_limit, int cost_lowerbound, int cost_upperbound)
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

  // print constraints;
  cout << "temporal cons:" << endl;
  for (auto cons: initial_priorities){
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = cons;
    auto task_from = id2task[a];
    auto task_to = id2task[x];
    cout << "precedence from " << task_from.first << "(" << task_from.second <<") to " << task_to.first << "(" << task_to.second << ")" << endl;
  }


	generateRoot();
  if (dummy_start->is_solution){
    solution_found = true;
    join_paths();
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

    // curr->conflict = chooseConflict(*curr);
    
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
				if (child[i]->is_solution) //no conflicts
          {// found a solution (and finish the while look)
            solution_found = true;
            solution_cost = child[i]->g_val;
            goal_node = child[i];
            updatePaths(child[i]);
            join_paths();
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


string PBS::getSolverName() const
{
  return "PBS";
}

void PBS::join_paths(){
  cout << "join path" << endl;
  joined_paths.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i ++){
    for (int j = 0; j < search_engines[i]->goal_location.size(); j++){
      int task_id = task2id({i, j});
      if (j == 0){
        joined_paths[i].path.push_back(paths[task_id]->front());
      }

      assert(joined_paths[i].size() - 1 == paths[task_id]->begin_time);
      for (int k = 1; k < paths[task_id]->size(); k++){
        joined_paths[i].path.push_back(paths[task_id]->at(k));
      }
      joined_paths[i].timestamps.push_back(joined_paths[i].size() - 1);
    }
  }
  paths.resize(num_of_agents);
  for (int i = 0 ; i < num_of_agents; i++){
    paths[i] = & joined_paths[i];
  }
}



PBS::~PBS(){
	releaseNodes();
	mdd_helper.clear();
}

