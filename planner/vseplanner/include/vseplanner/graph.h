#ifndef GRAPH_H_
#define GRAPH_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

namespace gbplanner {

class Graph {
public:
  // Ref: .../using_adjacency_list.html#sec:choosing-graph-type
  typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS,
                                boost::property<boost::vertex_index_t, int>,
                                boost::property<boost::edge_weight_t, double>,
                                boost::no_property>
      GraphType;
  // "using" as an alias; "typename" to explicitly claim that GraphType is a
  // type not class/variable...
  using VertexDescriptor = typename GraphType::vertex_descriptor;
  using EdgeDescriptor = typename GraphType::edge_descriptor;
  using EdgeDescriptorPair = typename std::pair<EdgeDescriptor, bool>;

  Graph();
  ~Graph();

  VertexDescriptor addSourceVertex(int id);
  VertexDescriptor addVertex(int id);
  EdgeDescriptorPair addEdge(int u_id, int v_id, double weight);
  bool findDijkstraShortestPaths();
  int getVertexID(VertexDescriptor v);
  int getNumVertices();
  int getNumEdges();
  // Get shortest path from this vertex to the source vertex
  void getShortestPath(int id, std::vector<int> &path);
  void printResults();
  void clear();

private:
  GraphType graph_;
  VertexDescriptor source_;
  std::map<int, VertexDescriptor> vertex_descriptors_;
  std::vector<VertexDescriptor> vertices_;
  std::vector<double> shortest_distances_;
  std::vector<VertexDescriptor> shortest_paths_;
  int num_vertices_;
};
}

#endif
