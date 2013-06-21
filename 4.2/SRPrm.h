#ifndef SRPRM_H
#define SRPRM_H

#include "basic_typedef.h"
#include "Graph.h"
#include "SRCollisionDetector.h"
#include "SRSampler.h"
#include "Kd_tree.h"
#include <map>

// Should be used with the Graph wrapper
struct Less_than_int
{
  bool operator()(const int& p1, const int& p2) const 
  {
    return (p1 < p2);
  }
};

class SRPrm {
public:
  /* IMPORTANT: The main code of the SRPrm algorithm should be placed in this
   * file.
   * You may change its structure, but make sure you that it remains consistent
   * with the run() operation in Planner.h
   */
  SRPrm(int number_vertices, 
      int k_nearest, double epsilon,
      const SRCollisionDetector& col, 
      const SRSampler &sampler,
      Point_2 start, Point_2 target);

  ~SRPrm();

  void add_edges( Point_2 p, int K, double EPS );

  //  This operation is supposed to generate the roadmap (sample configurations
  //  and connect them.
  void generate_roadmap();

  //  Returns a point path from start to target.
  //  If a path doesn't exist, returns empty vector.
  const vector<Point_2> &retrieve_path() const
  {
	return m_path;
  }

  void get_neighbors(const Point_2& pt, std::back_insert_iterator<std::vector<Point_2> > it) const;
  Point_2 get_nearest_to(const Point_2 &pt) const;

private:
  typedef Graph<int, Less_than_int> graph_t;
  int m_number_vertices;                // number of sampled vertices
  int m_k_nearest;                      // maximal degree of vertex in roadmap
  const SRCollisionDetector& m_col;
  const SRSampler& m_sampler;
  graph_t* m_graph;   //  Graph structure
  Kd_tree_2<Kernel>* m_kd_tree;         //  Kd-tree for nearest neighbor search
  Point_2 m_start,m_target;             //  Start and target configurations
  map< Point_2, int > vertexID;			//  mapping point to node id
  map< int, Point_2 > vertexIDToPoint;  // mapping node id to point
  bool is_path;                         // is there a path from m_start to m_target
  vector<Point_2> m_path;				// the result path
  double m_epsilon;
};

#endif
