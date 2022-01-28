#include "STPHelper.h"

void STPHelper::propagate_root(CBSNode* root, vector<ConstraintTable>& initial_constraints){
  if (!enabled){
    return;
  }

  clock_t t = clock();

  // generate STP only with distance heuristic
  agent_lb_ub.resize(search_engines.size());
  for (int i = 0; i < search_engines.size(); i++){
    agent_lb_ub[i].resize(search_engines[i]->goal_location.size(), {0, INT_MAX});
    agent_lb_ub[i][0].first = search_engines[i]->my_heuristic[0][search_engines[i]->start_location];
    tp.add_node(std::to_string(i) + "_" + std::to_string(0));
    tp.add_lb(std::to_string(i) + "_" + std::to_string(0), search_engines[i]->my_heuristic[0][search_engines[i]->start_location]);

    for (int j = 1; j < search_engines[i]->goal_location.size(); j++){
      tp.add_node(std::to_string(i) + "_" + std::to_string(j));
      tp.add_edge(std::to_string(i) + "_" + std::to_string(j),
                  std::to_string(i) + "_" + std::to_string(j - 1),
                  - search_engines[i]->my_heuristic[j][search_engines[i]->goal_location[j-1]]
                  );
      agent_lb_ub[i][j].first = agent_lb_ub[i][j - 1].first +
        search_engines[i]->my_heuristic[j][search_engines[i]->goal_location[j-1]];
    }
  }
  // test

  distance_matrix_t dm;
  auto res  = compute_distance(tp, dm);
  assert(res);

  for (int i = 0; i < search_engines.size(); i++){
    for (int j = 1; j < search_engines[i]->goal_location.size(); j++){
      string name = std::to_string(i) + "_" + std::to_string(j);
      assert(get_lb(tp, dm , name) == agent_lb_ub[i][j].first && get_ub(tp, dm , name) == agent_lb_ub[i][j].second);
    }
  }

  //
  int num_of_agents = search_engines.size();
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = 0; a2 < num_of_agents; a2++){
      if (a1 == a2){
        continue;
      }
      for (auto cons: search_engines[0]->instance.temporal_cons[a1 * num_of_agents + a2]){
        auto from_landmark = cons.first;
        auto to_landmark = cons.second;

        cout << "add con" <<
          a1 << "("  << from_landmark<< ")"
             << a2 << "("  << to_landmark<< ")" << endl;
        // we should have (paths[a1]->timestamps[from_landmark] < paths[a2]->timestamps[to_landmark])
        tp.add_edge(std::to_string(a2) + "_" + std::to_string(to_landmark),
                    std::to_string(a1) + "_" + std::to_string(from_landmark),
                    -1);
      }
    }
  }

  res  = compute_distance(tp, dm);
  assert(res);

  for (int i = 0; i < search_engines.size(); i++){
    initial_constraints[i].leq_goal_time.resize(search_engines[i]->goal_location.size(), INT_MAX);
    initial_constraints[i].g_goal_time.resize(search_engines[i]->goal_location.size(), -1);
    for (int j = 0; j < search_engines[i]->goal_location.size(); j++){
      string name = std::to_string(i) + "_" + std::to_string(j);
      if(get_lb(tp, dm , name) != agent_lb_ub[i][j].first){
        cout << "new lb for " << name << ": " << get_lb(tp, dm , name) << endl;
        initial_constraints[i].g_goal_time[j] = get_lb(tp, dm , name) - 1;
        initial_constraints[i].length_min = max(initial_constraints[i].length_min, get_lb(tp, dm , name));
      }
      if(get_ub(tp, dm , name) != agent_lb_ub[i][j].second){
        cout << "new ub for " << name << ": " << get_ub(tp, dm , name) << endl;
        initial_constraints[i].leq_goal_time[j] = get_ub(tp, dm , name);
      }
    }
  }



  accumulated_runtime += (double) (clock() - t) / CLOCKS_PER_SEC;
}

bool STPHelper::propagate(CBSNode* parent, CBSNode* child){
  if (!enabled){
    return true;
    
  }

	clock_t t = clock();

  auto tp_copy = tp;
  auto curr = parent;

  while (curr->parent !=nullptr){
		int a, x, y, t;
		constraint_type type;
		tie(a, x, y, t, type) = curr->constraints.front();
    if (type == constraint_type::GSTOP || type == constraint_type::LEQSTOP){
      for (auto con: curr->constraints){
        tie(a, x, y, t, type) = con;
        if (type == constraint_type::LEQSTOP){
          tp_copy.add_ub(std::to_string(a) + "_" + std::to_string(x), t);
        }
        if (type == constraint_type::GSTOP ){

          tp_copy.add_lb(std::to_string(a) + "_" + std::to_string(x), t + 1);
        }

      }
    }
    curr= curr->parent;
  }

  if (prev_parent == parent){
    // cout << "calc skipped" << std::endl;
  } else{
    auto res  = compute_distance(tp_copy, dm_parent);
    assert(res);
    prev_parent = parent;
  }


  for (auto con: child->constraints){
		int a, x, y, t;
		constraint_type type;
    tie(a, x, y, t, type) = con;
    if (type == constraint_type::LEQSTOP){
      tp_copy.add_ub(std::to_string(a) + "_" + std::to_string(x), t);
    }
    if (type == constraint_type::GSTOP ){

      tp_copy.add_lb(std::to_string(a) + "_" + std::to_string(x), t + 1);
    }
  }

  distance_matrix_t dm_ch;
  auto res  = compute_distance(tp_copy, dm_ch);

  if (!res){
    cout << "STP not solvable" << std::endl;
    return false;
  }

  for (int i = 0; i < search_engines.size(); i++){
    // initial_constraints[i].leq_goal_time.resize(search_engines[i]->goal_location.size(), INT_MAX);
    // initial_constraints[i].g_goal_time.resize(search_engines[i]->goal_location.size(), -1);
    for (int j = 0; j < search_engines[i]->goal_location.size(); j++){
      string name = std::to_string(i) + "_" + std::to_string(j);
      if(get_lb(tp_copy, dm_parent , name) != get_lb(tp_copy, dm_ch, name)){
        cout << "new lb for " << name << ": " << get_lb(tp, dm_ch, name) << endl;
        child->constraints.push_back({i, j, -1, get_lb(tp, dm_ch, name) -1, constraint_type::GSTOP});
        // initial_constraints[i].g_goal_time[j] = get_lb(tp, dm , name) - 1;
        // initial_constraints[i].length_min = max(initial_constraints[i].length_min, get_lb(tp, dm , name));
      }
      if(get_ub(tp_copy, dm_parent , name) != get_ub(tp_copy, dm_ch, name)){
      // if(get_ub(tp, dm , name) != agent_lb_ub[i][j].second){
        cout << "new ub for " << name << ": " << get_ub(tp, dm_ch , name) << endl;
        // initial_constraints[i].leq_goal_time[j] = get_ub(tp, dm , name);
        child->constraints.push_back({i, j, -1, get_ub(tp, dm_ch, name) -1, constraint_type::LEQSTOP});
      }
    }
  }


  
  accumulated_runtime += (double) (clock() - t) / CLOCKS_PER_SEC;
  return true;
}
