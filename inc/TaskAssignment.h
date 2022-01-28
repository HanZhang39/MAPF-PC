#pragma once

#include "SpaceTimeAStar.h"
#include "Instance.h"


class TaskAssignment: public Instance {

public:
  TaskAssignment(const string& map_fname, const string& agent_fname,
                 int num_of_agents = 0);
  void find_greedy_plan();

protected:
  // bool loadMap();
  int num_of_tasks;

  std::unique_ptr<SingleAgentSolver> search_engine;


  vector<int> task_locations;
  vector<vector<int>> task_plan;


  vector<pair<int,int>> temporal_dependecies;
  bool loadAgents();
};
