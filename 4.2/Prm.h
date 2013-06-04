#ifndef PRM_H
#define PRM_H

#include "basic_typedef.h"
#include "Graph.h"
#include "CollisionDetector.h"
#include "Sampler.h"
#include "Kd_tree_d.h"
#include <map>

// Should be used with the Graph wrapper


struct Less_than_int
{
  bool operator()(const int& p1, const int& p2) const 
  {
    return (p1 < p2);
  }
};

//struct point_d_less
//{
//	bool operator() (const Point_d& lhs, const Point_d& rhs)
//	{
//		if (lhs.dimension() < rhs.dimension()) return true;
//		if (rhs.dimension() < lhs.dimension()) return false;
//		
//		for(int i = 0; i < rhs.dimension(); ++i)
//		{
//			if (lhs.cartesian(i) < rhs.cartesian(i)) return true;
//			if (rhs.cartesian(i) < lhs.cartesian(i)) return false;
//		}
//		return false;
//	}
//};

class Prm {
	typedef map< Point_2, int > point_to_node_map_t;
public:
  /* IMPORTANT: The main code of the PRM algorithm should be placed in this
   * file.
   * You may change its structure, but make sure you that it remains consistent
   * with the run() operation in Planner.h
   */
  Prm(int number_vertices, 
      int k_nearest, 
      const CollisionDetector& col, 
      const Sampler& sampler,
      Point_2 start, Point_2 target); 



  ~Prm()
  {
  }

  double configuration_distance(const Point_d& lhs, const Point_d& rhs);
  void add_edges( const Point_d &p, int K, double EPS );
  
  //  This operation is supposed to generate the roadmap (sample configurations
  //  and connect them.
  void generate_roadmap();
  

  //  Returns a point path from start to target.
  //  If a path doesn't exist, returns empty vector.
  const vector<Point_2> &retrieve_path()
  {
	return m_path;
  }

  const Kd_tree_d<Kernel_d> &get_knn()
  {
	  return m_kd_tree;
  }

  std::pair<typename Graph<int, Less_than_int>::adjacency_iterator,
                     typename Graph<int, Less_than_int>::adjacency_iterator>
  get_neighbors(const Point_2& node) const
  {
	  point_to_node_map_t::const_iterator it = vertexID.find(node);
	  assert(it != vertexID.end());

	  int id = it->second;
	  return m_graph.get_neighbors(id);
  }

  Point_2 get_point_from_graph_id(unsigned int graph_id) const
  {
	  unsigned int node = *m_graph.node_by_id(graph_id);
	  return vertexIDToPoint.find(node)->second;
  }

private:
  int m_number_vertices;                // number of sampled vertices
  int m_k_nearest;                      // maximal degree of vertex in roadmap
  const CollisionDetector &m_col;
  const Sampler &m_sampler;
  Graph<int, Less_than_int> m_graph;   //  Graph structure
  Kd_tree_d<Kernel_d> m_kd_tree;         //  Kd-tree for nearest neighbor search
  Point_d m_start,m_target;             //  Start and target configurations
  point_to_node_map_t vertexID;			//  mapping point to node id
  map< int, Point_2 > vertexIDToPoint;  // mapping node id to point
  bool is_path;                         // is there a path from m_start to m_target
  vector<Point_2> m_path;				// the result path
};

#endif
