#include "stp/TemporalGraph.hpp"
#include <iostream>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/copy.hpp>
#include <cfloat>


TemporalGraph::TemporalGraph(TemporalGraph& graph_to_copy){
  boost::copy_graph(graph_to_copy.G, this->G);
  for (auto vd : boost::make_iterator_range(vertices(G))){
    if (G[vd].name == "x0"){
      x0 = vd;
    } else {
      vertex_map[G[vd].name] = vd;
    }
  }
}

std::pair<edge_t, bool> TemporalGraph::add_lb(std::string name, int lb){
  auto e = boost::edge(vertex_map[name],x0,G);
  if (!e.second){
    e = boost::add_edge(vertex_map[name], x0, G);
    G[e.first].edge_weight = -lb;
  }else{
    if (-G[e.first].edge_weight < lb){
      G[e.first].edge_weight = -lb;
    }
  }
  return e;
}

std::pair<edge_t, bool> TemporalGraph::add_ub(std::string name, int ub){
  auto e = boost::edge(x0, vertex_map[name],G);
  if (!e.second){
    e = boost::add_edge(x0, vertex_map[name], G);
    G[e.first].edge_weight = ub;
  } else {
      if (G[e.first].edge_weight > ub){
        G[e.first].edge_weight = ub;
      }
  }
  // e = boost::add_edge(x0, vertex_map[name], G);
  return e;
}


void TemporalGraph::add_ub(int ub){
  for (auto it: vertex_map){
    std::string name = it.first;
    add_ub(name, ub);
  }
}

bool TemporalGraph::add_node(std::string name){
  if (vertex_map.find(name) != vertex_map.end()){
    return false;
  }
  auto v = boost::add_vertex(G);
  G[v].name = name;
  vertex_map[name] = v;
  return true;
}


std::pair<edge_t, bool> TemporalGraph::add_edge(std::string from, std::string to, int ub){
  auto fromIter = vertex_map.find(from);
  auto toIter = vertex_map.find(to);

  if (fromIter == vertex_map.end()
      || toIter == vertex_map.end()
      || fromIter->second == toIter->second) {
    std::cerr << "invalid edge! " << std::endl;
  }
  auto e = boost::edge(fromIter->second, toIter->second, G);
  if (!e.second){
    e = boost::add_edge(fromIter->second, toIter->second, G);
    // auto e = boost::add_edge(fromIter->second, toIter->second, G);
    G[e.first].edge_weight = ub;
  }else{
    if (ub < G[e.first].edge_weight){
      G[e.first].edge_weight = ub;
    }
  }

  return e;
}

void TemporalGraph::print_graph(){
  std::cout << "Vertices " << std::endl;
  for (auto vd : boost::make_iterator_range(vertices(G))){
    std::cout << "vertex " << (int) vd << ": " << G[vd].name << std::endl;
  }
  std::cout << "\nEdges: " << std::endl;
  auto es = boost::edges(G);
  for (auto eit = es.first; eit != es.second; ++eit) {
    std::cout << boost::source(*eit, G) << ' ' << boost::target(*eit, G) << " ub: " << G[*eit].edge_weight << std::endl;
  }
}




schedule_t to_schedule(const TemporalGraph&tg, distance_matrix_t& distance_matrix){
  schedule_t map;
  for (auto it:tg.vertex_map){
    std::string name = it.first;
    map[name] = get_lb(tg, distance_matrix, name);
  }
  return map;
}


int makespan(const TemporalGraph&tg, distance_matrix_t& distance_matrix){
  int t = 0;
  for (auto it:tg.vertex_map){
    std::string name = it.first;
    t = std::max(t, get_lb(tg, distance_matrix, name));
  }
  return t;
}



bool compute_distance(const TemporalGraph& tg, distance_matrix_t& distance_matrix){
  bool r = boost::floyd_warshall_all_pairs_shortest_paths(tg.G, distance_matrix, boost::weight_map(boost::get(&TemporalEdge::edge_weight, tg.G)));

  if (!r){
    std::cout << "Unsuccess" << std::endl;
    return false;
  }
  return true;
}

int get_dist(const TemporalGraph& tg, const distance_matrix_t matrix, const std::string name_0, const std::string name_1){
  return matrix.at(tg.vertex_map.at(name_0)).at(tg.vertex_map.at(name_1));
}

int get_lb(const TemporalGraph& tg, const distance_matrix_t& matrix, const std::string name){
  return - matrix.at(tg.vertex_map.at(name)).at(tg.x0);
}

int get_ub(const TemporalGraph& tg, const distance_matrix_t& matrix, const std::string name){
  return matrix.at(tg.x0).at(tg.vertex_map.at(name));
}
