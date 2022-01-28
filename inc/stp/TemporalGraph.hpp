#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>


typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::bidirectionalS > searchGraphTraits_t;
typedef searchGraphTraits_t::vertex_descriptor vertex_t;
typedef searchGraphTraits_t::edge_descriptor edge_t;


struct TemporalEdge
{
  // We only model upper bound for simplicity;
  int edge_weight;
};


struct TemporalNode
{
  std::string name;
};


typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, TemporalNode, TemporalEdge> TemporalGraph_t;

typedef boost::unordered_map<vertex_t, boost::unordered_map<vertex_t, int> > distance_matrix_t;

typedef std::unordered_map<std::string, int> schedule_t;


class TemporalGraph
{
private:
  // Shortest length
  // boost::unordered_map<vertex_t, boost::unordered_map<vertex_t, double> > distance_matrix;

public:

  TemporalGraph(TemporalGraph& graph_to_copy);
  vertex_t x0;
  boost::unordered_map<std::string, vertex_t> vertex_map;
  TemporalGraph_t G;

  // clean shortest distance stored
  // void clear_distance();

  // TODO change name to compute_pairwise_distance
  // void compute_distance();

  TemporalGraph(){
    x0 = boost::add_vertex(G);
    G[x0].name = "x0";
  };
  ~TemporalGraph(){};
  // double get_dist(const std::string&, const std::string&) const;

  // double get_ub(const std::string&) const;
  // double get_lb(const std::string&) const;

  std::pair<edge_t, bool> add_lb(std::string name, int lb);
  std::pair<edge_t, bool> add_ub(std::string name, int ub);

  // add ub to every node
  void add_ub(int ub);

  bool add_node(std::string name);
  std::pair<edge_t, bool> add_edge(std::string from, std::string to, int ub);

  void print_graph();


};

schedule_t to_schedule(const TemporalGraph&tg, distance_matrix_t& distance_matrix);

int makespan(const TemporalGraph&tg, distance_matrix_t& distance_matrix);

bool compute_distance(const TemporalGraph& tg, distance_matrix_t& distance_matrix);
int get_dist(const TemporalGraph& tg, const distance_matrix_t matrix, const std::string name_0, const std::string name_1);
int get_lb(const TemporalGraph& tg, const distance_matrix_t& matrix, const std::string name);
int get_ub(const TemporalGraph& tg, const distance_matrix_t& matrix, const std::string name);
