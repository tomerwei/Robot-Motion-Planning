#ifndef GRAPH_H
#define GRAPH_H

#include "Connected_components.h"
#include "basic_typedef.h"
#include <boost/graph/adjacency_list.hpp>

////////////////////////////////////////////////////////////////////////////
// The graph is templated
// weights: the weight each edges has (currently not used)
////////////////////////////////////////////////////////////////////////////


template <typename Node, typename Is_Less_than = std::less<Node> >
class Graph {
protected:
  // a weight is currently an int
  typedef boost::property<boost::edge_weight_t, double>       Weight_property;

  typedef boost::adjacency_list<boost::vecS,          //container type for the out edge list
                                boost::vecS,          //container type for the vertex list
                                boost::undirectedS,   //directed / undirected
                                int,                  //vertex properties
                                Weight_property       //edge properties
                                >  Boost_graph;       //slightly more time efficient
  typedef boost::graph_traits<Boost_graph>::vertex_descriptor Vertex_descriptor;
  typedef boost::graph_traits<Boost_graph>::edge_iterator     Edge_iterator;
  typedef boost::graph_traits<Boost_graph>::edge_descriptor   EdgeBoost;

protected:
  Boost_graph* _graph;
  std::vector<Vertex_descriptor>  _parent;
  std::vector<std::vector<Vertex_descriptor> > _next_vertex_to_target; 
  std::vector<double> _distance;
  std::vector<std::vector<double> >  _distance_to_target;
  std::vector<int> _component; 
  std::map<int, Node> _id_node_map;
  typedef std::map<Node, int, Is_Less_than> node_map_t;
  node_map_t _node_id_map;
  int _curr_id;
  
protected:
  bool                  _use_connected_components;
  Connected_components  _connected_components;
        
  /* this method will run Dijkstra on the current graph representation from
   * the source input argument. The method can fail if the source is not the
   * in the graph, return value indicates success or failure. Upon success
   * this method updates the parent and distance data members.
   */
  void run_dijkstra(const int source_id)
  {
    int num_of_vertices(boost::num_vertices(*_graph));

    //will hold for each vertex its parent in the shortest path to source.
    std::vector<Vertex_descriptor> parent(num_of_vertices);

    //will hold for each vertex its distance in the shortest path to source.
    std::vector<double> distance(num_of_vertices);
    Vertex_descriptor source_descriptor(source_id);
            
    //stores result at parent and distance.
    dijkstra_shortest_paths(*_graph, source_descriptor,
                            boost::predecessor_map(&parent[0]).
                            distance_map(&distance[0]));
    this->_parent=parent;
    this->_distance=distance;
  }

public:
  typedef Boost_graph::adjacency_iterator adjacency_iterator;
  Graph(int initial_size = 0, int use_connected_components = false) :
    _curr_id(0), _use_connected_components(use_connected_components)
  {
    _graph = new Boost_graph(initial_size);
    int i = 0;
  }

  ~Graph(void)
  {
    delete _graph;
  }
        
  /* add vertex to the graph. the vertex property is its ID. */
  bool is_in_graph(const Node& node)
  {
    return ((_node_id_map.find(node) != _node_id_map.end()) ? true : false);
  }

  void add_vertex(const Node& node)
  {
    if (is_in_graph(node)) return; //node is in the graph

    int id = get_unique_id();
    _node_id_map[node]  = id;
    _id_node_map[id]    = node;

    add_vertex_to_graph (id);
    if (_use_connected_components) _connected_components.add_element(id);
  }

  void add_edge(const Node& node1, const Node& node2, const double weight=1)
  {
    CGAL_precondition (is_in_graph(node1)); //node1 is in the graph
    CGAL_precondition (is_in_graph(node2)); //node2 is in the graph
    
    int id1 (_node_id_map[node1]);
    int id2 (_node_id_map[node2]);

    add_edge_to_graph (id1,id2,weight);
    if (_use_connected_components)
      _connected_components.connect_elements(id1,id2);
  }
  
  /* this method runs Dijkstra algorithm on the graph. */
  void find_path(const Node& source, const Node& target, std::list<Node>& path)
  {
    CGAL_precondition (is_in_graph(source)); //source is in the graph
    CGAL_precondition (is_in_graph(target)); //target is in the graph

    int source_id (_node_id_map[source]);
    int target_id (_node_id_map[target]);

    if (_use_connected_components && !_is_in_same_cc(source_id,target_id))
      return;

    run_dijkstra(source_id);
    int curr_node_id =target_id;

    // if target configuration parent is itself, there is no path from source
    // to target.
    if (_parent[curr_node_id]==curr_node_id) return;

    CGAL_precondition (_id_node_map.find(curr_node_id) != _id_node_map.end());
    path.push_front(_id_node_map[curr_node_id]);

    while(_parent[curr_node_id] != curr_node_id )
    {
      curr_node_id = _parent[curr_node_id];
      CGAL_precondition (_id_node_map.find(curr_node_id) != _id_node_map.end());
      path.push_front (_id_node_map[curr_node_id]);
    }
  }

  /* this method updates calculates the distance of all nodes from the given
   * target vertex, as well the actual path
   */
  void update_paths_to_target(const Node& target)
  {
    CGAL_precondition (is_in_graph(target)); //target is in the graph

    int target_id (_node_id_map[target]);
    run_dijkstra(target_id);

    _next_vertex_to_target.push_back(_parent);
    _distance_to_target.push_back(_distance);
  }

  void clear_paths_to_target()
  {
    _next_vertex_to_target.clear();
    _distance_to_target.clear();
  }

  /* retuns distance from given node to target */
  double distance_to_target(const Node& node, int robot)
  {
    int node_id (_node_id_map[node]);
    return _distance_to_target[robot][node_id];
  }

  /* retuns the neighbor of node that is on the optimal path to target */
  Node* optimal_neighbor(const Node& node, int robot)
  {
    int node_id (_node_id_map[node]);
    int neighbor_id = _next_vertex_to_target[robot][node_id];
    return &(_id_node_map[neighbor_id]);
  }

  Node* optimal_neighbor(const Node& node1, const Node& node2, int robot)
  {
    list<Node> path;
    find_path(node1, node2, path);
    assert(path.size() > 1);
                
    Node next_optimal = path[1];
    return &next_optimal;
  }

  Node* node_by_id(int id) const
  {
	  return this->_id_node_map[id];
  }

  void get_neighbors(const Node& node, std::back_insert_iterator< std::vector < Node > > out) const
  {
	  typename node_map_t::const_iterator it = _node_id_map.find(node);
	  assert(it != _node_id_map.end());

	  int id = it->second;
	  std::pair<adjacency_iterator, adjacency_iterator> p = adjacent_vertices(id, *_graph);

	  adjacency_iterator n_it = p.first,
		it_end = p.second;
	  for(; n_it != it_end; ++n_it)
	  {
		  *++out = _id_node_map.find(*n_it)->second;
	  }
  }

  // cc utils
  bool is_in_same_cc(const Node& source, const Node& target)
  {
    CGAL_precondition (_use_connected_components);
    CGAL_precondition (is_in_graph(source)); //source is in the graph
    CGAL_precondition (is_in_graph(target)); //target is in the graph

    int source_id (_node_id_map[source]);
    int target_id (_node_id_map[target]);

    return _is_in_same_cc(source_id,target_id);
  }

  int get_cc_id (const Node& node)
  {
    CGAL_precondition (_use_connected_components);
    CGAL_precondition (is_in_graph(node)); //source is in the graph
    return _connected_components.get_connected_component_id(_node_id_map[node]);
  }

  std::vector<std::vector<Node> > get_connected_components()
  {
    CGAL_precondition (_use_connected_components);
    Int_cc_map& ccs_ids = _connected_components.get_connected_components();
    Int_cc_map::iterator iter;

    std::vector<std::vector<Node> > ccs_nodes;
    for (iter = ccs_ids.begin(); iter!= ccs_ids.end(); ++iter) {
      std::vector<Node> cc;
      BOOST_FOREACH(int i, iter->second) cc.push_back(_id_node_map[i]);
      ccs_nodes.push_back(cc);
    }
    return ccs_nodes;
  }

  void print_connected_components()
  {
    _connected_components.print_connected_components();
    return;
  }

  //debug
  void print() const 
  {
    // get the property map for vertex indices
    typedef boost::property_map<Boost_graph,
                                boost::vertex_index_t>::type Index_map;
    Index_map index = get(boost::vertex_index, *_graph);

    //print vertices
    std::cout << "graph vertices :" << std::endl;
    typedef boost::graph_traits<Boost_graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(*_graph); vp.first != vp.second; ++vp.first)
      std::cout << index[*vp.first] <<  " ";
    std::cout << std::endl;

    //print edges
    std::cout << "graph edges : ";
    boost::graph_traits<Boost_graph>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(*_graph); ei != ei_end; ++ei)
      std::cout << "(" << index[source(*ei, *_graph)] 
                << "," << index[target(*ei, *_graph)] << ") ";
    std::cout << std::endl;
    return;
  }

protected:
  int get_unique_id()
  {
    return _curr_id++;
  }

  void add_vertex_to_graph(int node_id)
  {
    boost::add_vertex(node_id, *_graph);
  }

  void add_edge_to_graph(const int id1,const int id2, const double weight = 1)
  {
    boost::property<boost::edge_weight_t, double> edge_property(weight);
    boost::add_edge(id1, id2,edge_property, *_graph);        
  }

  bool _is_in_same_cc(const int id1,const int id2)
  {
    CGAL_precondition(_use_connected_components);
    return (_connected_components.is_in_same_connected_component(id1, id2));
  }
};  //Graph

#endif  //GRAPH_H
