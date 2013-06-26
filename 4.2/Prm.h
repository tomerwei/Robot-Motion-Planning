#ifndef PRM_H
#define PRM_H

#include "basic_typedef.h"
#include "Graph.h"
#include "CollisionDetector.h"
#include "Sampler.h"
#include "Kd_tree_d.h"
#include "SRPrm.h"
#include <map>
#include <utility>

// Should be used with the Graph wrapper


class Prm {
	typedef map< Point_d, int, point_d_less > point_to_node_map_t;
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
      Point_d start, Point_d target); 



  ~Prm()
  {
    delete m_graph;
  }

  double configuration_distance(const Point_d& lhs, const Point_d& rhs);
  void add_edges( const Point_d &p, int K );
  
  //  This operation is supposed to generate the roadmap (sample configurations
  //  and connect them.
  void generate_roadmap();
  

  //  Returns a point path from start to target.
  //  If a path doesn't exist, returns empty vector.
  const vector<Point_d>& retrieve_path() const
  {
	return m_path;
  }

private:
  int m_number_vertices;                // number of sampled vertices
  int m_k_nearest;                      // maximal degree of vertex in roadmap
  const CollisionDetector &m_col;
  const Sampler &m_sampler;
  LocalPlanner m_loc_planner;
  Graph<int, Less_than_int>* m_graph;   //  Graph structure
  Kd_tree_d<Kernel_d> m_kd_tree;         //  Kd-tree for nearest neighbor search
  Point_d m_start,m_target;             //  Start and target configurations
  point_to_node_map_t vertexID;			//  mapping point to node id
  map< int, Point_d > vertexIDToPoint;  // mapping node id to point
  bool is_path;                         // is there a path from m_start to m_target
  vector<Point_d> m_path;				// the result path
};

#endif
