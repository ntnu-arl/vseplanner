#include "vseplanner/graph.h"

namespace gbplanner {

Graph::Graph() {}
Graph::~Graph() {}

Graph::VertexDescriptor Graph::addSourceVertex(int id) {
  source_ = boost::add_vertex(id, graph_);
  vertex_descriptors_[id] = source_;
  return source_;
}

Graph::VertexDescriptor Graph::addVertex(int id) {
  VertexDescriptor v = boost::add_vertex(id, graph_);
  vertex_descriptors_[id] = v;
  return v;
}

Graph::EdgeDescriptorPair Graph::addEdge(int u_id, int v_id, double weight) {
  if (vertex_descriptors_.find(u_id) == vertex_descriptors_.end()) {
    addVertex(u_id);
  }
  if (vertex_descriptors_.find(v_id) == vertex_descriptors_.end()) {
    addVertex(v_id);
  }
  return boost::add_edge(vertex_descriptors_[u_id], vertex_descriptors_[v_id],
                         weight, graph_);
}

bool Graph::findDijkstraShortestPaths() {
  num_vertices_ = boost::num_vertices(graph_);
  if (num_vertices_ < 2)
    return false;

  shortest_distances_.resize(num_vertices_);
  shortest_paths_.resize(num_vertices_);

  auto v_index = boost::get(boost::vertex_index, graph_);
  auto weight = boost::get(boost::edge_weight, graph_);
  boost::dijkstra_shortest_paths(
      graph_, source_,
      boost::predecessor_map(
          boost::make_iterator_property_map(shortest_paths_.begin(),
                                            get(boost::vertex_index, graph_)))
          .distance_map(boost::make_iterator_property_map(
              shortest_distances_.begin(), get(boost::vertex_index, graph_))));

  return true;
}

int Graph::getVertexID(VertexDescriptor v) {
  return boost::get(boost::get(boost::vertex_index, graph_), v);
}

int Graph::getNumVertices() { return boost::num_vertices(graph_); }

int Graph::getNumEdges() { return boost::num_edges(graph_); }

void Graph::getShortestPath(int id, std::vector<int> &path) {
  if (shortest_paths_.size() == 0) {
    std::cout << "[ERROR] Shortest paths are not calculated yet" << std::endl;
    return;
  }

  path.clear();
  int parent_id = getVertexID(shortest_paths_[id]);
  if ((id == 0) || (parent_id == id)) {
    // do nothing since this is source vertex or isolated vertex
    return;
  } else {
    path.push_back(id); // vertex id first
    path.push_back(parent_id); // its first parent, the rest is recursively looked up
    while (parent_id != 0) {
      parent_id = getVertexID(shortest_paths_[parent_id]);
      path.push_back(parent_id);
    }
  }
}

void Graph::printResults() {
  std::cout << "Shortest paths:" << std::endl;
  for (int i = 0; i < num_vertices_; ++i) {
    int id = getVertexID(shortest_paths_[i]);
    std::cout << "Path: "
              << "[cost: " << shortest_distances_[i] << "] " << i << "<-" << id;
    if (id == 0) {
    } else if (id == i)
      std::cout << "[isolated-vertex]";
    else {
      int id_parent = id;
      while (id_parent != 0) {
        id_parent = getVertexID(shortest_paths_[id_parent]);
        std::cout << "<-" << id_parent;
      }
    }
    std::cout << std::endl;
  }
}

}
