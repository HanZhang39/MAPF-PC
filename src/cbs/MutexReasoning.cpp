#include "MutexReasoning.h"
#include "ConstraintPropagation.h"


shared_ptr<Conflict> MutexReasoning::run(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2)
{
  if (a1 > a2){
    std::swap(a1, a2);
    std::swap(mdd_1, mdd_2);
  }

  if (strategy == mutex_strategy::N_MUTEX){
    return nullptr;
  }

	clock_t t = clock();
	auto conflict = findMutexConflict(paths, a1, a2, node, mdd_1, mdd_2);
	accumulated_runtime += (double) (clock() - t) / CLOCKS_PER_SEC;
	return conflict;
}

shared_ptr<Conflict> MutexReasoning::findMutexConflict(const vector<Path*> & paths, int a1, int a2, CBSNode& node, MDD* mdd_1, MDD* mdd_2){
  assert(a1 < a2);
	ConstraintsHasher c_1(a1, &node);
	ConstraintsHasher c_2(a2, &node);
  bool use_general_mutex = false;//(strategy == mutex_strategy::MUTEX_NC_FIRST_K) || ( strategy == mutex_strategy::MUTEX_NC_GREEDY ) || (strategy == mutex_strategy::MUTEX_NC_GREEDY_F);

  if (has_constraint(c_1, c_2)){
    if (!use_general_mutex){
      return find_applicable_constraint(c_1, c_2, paths);
    }
    else{
      shared_ptr<Conflict> mutex_conflict = find_applicable_constraint(c_1, c_2, paths);
      if (mutex_conflict != nullptr){
        // found a usable mutex;
        return mutex_conflict;
      }
    }
  }

  ConstraintPropagation* cp;
  shared_ptr<Conflict> mutex_conflict = nullptr;

  cp = new ConstraintPropagation(mdd_1, mdd_2);
  cp->init_mutex();
  cp->fwd_mutex_prop();

  if (cp->_feasible(mdd_1->levels.size() - 1, mdd_2->levels.size() - 1) >= 0){
    delete cp;
    cache_constraint(c_1, c_2, nullptr);
    return nullptr;
  }

  delete cp;

  // generate constraint;
  mutex_conflict = make_shared<Conflict>();
 mutex_conflict->mutexConflict(a1, a2);

  MDD mdd_1_cpy(*mdd_1);
  MDD mdd_2_cpy(*mdd_2);

  ConstraintTable ct1(initial_constraints[a1]);
  ConstraintTable ct2(initial_constraints[a2]);

  ct1.build(node, a1, search_engines[a1]->goal_location.size());
  ct2.build(node, a2, search_engines[a2]->goal_location.size());
  auto ip = IPMutexPropagation(&mdd_1_cpy, &mdd_2_cpy, search_engines[a1], search_engines[a2],
                               ct1, ct2);
  con_vec a;
  con_vec b;
  std::tie(a, b) = ip.gen_constraints();

  for (auto con:a){
    std::get<0>(con) = a1;
    mutex_conflict->constraint1.push_back(con);
  }

  for (auto con:b){
    std::get<0>(con) = a2;
    mutex_conflict->constraint2.push_back(con);
  }

  mutex_conflict->final_len_1 = ip.final_len_0;
  mutex_conflict->final_len_2 = ip.final_len_1;

  // mutex_conflict->constraint1 = list<Constraint>(a.begin(), a.end());
  // mutex_conflict->constraint2 = list<Constraint>(b.begin(), b.end());

  cache_constraint(c_1, c_2, mutex_conflict);

  // prepare for return
  return mutex_conflict;
}

bool MutexReasoning::constraint_applicable(const vector<Path*> & paths, shared_ptr<Conflict> conflict){
  if (conflict->priority== conflict_priority::CARDINAL){
    return true;
  }else{
    return constraint_applicable(paths, conflict->constraint1) && constraint_applicable(paths, conflict->constraint2);
  }
}


bool MutexReasoning::constraint_applicable(const vector<Path*> & paths, list<Constraint>& constraint){
  for (Constraint& c: constraint){
    if (std::get<4>(c) == constraint_type::VERTEX){
      int ag = std::get<0>(c);
      int loc = std::get<1>(c);
      int t = std::get<3>(c);
      if ((*paths[ag])[t].location == loc) {
        return true;
      }
    }else if (std::get<4>(c) == constraint_type::EDGE){
      int ag = std::get<0>(c);
      int loc = std::get<1>(c);
      int loc_to = std::get<2>(c);
      int t = std::get<3>(c);
      if ((*paths[ag])[t - 1].location == loc &&
          (*paths[ag])[t].location == loc_to ) {
        return true;
      }
    }
  }
  return false;
}

void MutexReasoning::cache_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2, shared_ptr<Conflict> constraint){
  lookupTable[c1][c2].push_back(constraint);
}

shared_ptr<Conflict> MutexReasoning::find_applicable_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2, const vector<Path*> & paths){
  if (lookupTable.find(c1) != lookupTable.end()){
    if (lookupTable[c1].find(c2) != lookupTable[c1].end()){
      for (auto& constraint: lookupTable[c1][c2]){
        if ((strategy == mutex_strategy::MUTEX_C) && constraint == nullptr){
          return nullptr;
        }
        if (constraint_applicable(paths, constraint)){
          return make_shared<Conflict>(*constraint);
        }
      }
    }
  }
  return nullptr;
}

bool MutexReasoning::has_constraint(ConstraintsHasher & c1, ConstraintsHasher & c2){
  return lookupTable.find(c1) != lookupTable.end() && lookupTable[c1].find(c2) != lookupTable[c1].end();
}

