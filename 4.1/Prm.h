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

struct point_d_less
{
	bool operator() (const Point_d& lhs, const Point_d& rhs)
	{
		if (lhs.dimension() < rhs.dimension()) return true;
		if (rhs.dimension() < lhs.dimension()) return false;
		
		for(int i = 0; i < rhs.dimension(); ++i)
		{
			if (lhs.cartesian(i) < rhs.cartesian(i)) return true;
			if (rhs.cartesian(i) < lhs.cartesian(i)) return false;
		}
		return false;
	}
};

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
      CollisionDetector* col, 
      Sampler* sampler,
      Point_d start, Point_d target) 
:m_number_vertices(number_vertices)              // number of sampled vertices
,m_k_nearest(k_nearest)                      // maximal degree of vertex in roadmap
,m_col(col)
,m_sampler(sampler)
,m_graph(NULL)   //  Graph structure
,m_kd_tree(NULL)         //  Kd-tree for nearest neighbor search
,m_start(start)
,m_target(target)             //  Start and target configurations
,vertexID()			//  mapping point to node id
,vertexIDToPoint()  // mapping node id to point
,is_path(false)                         // is there a path from m_start to m_target
,m_path()				// the result path
  {

  }


  ~Prm()
  {
    delete m_graph;
    delete m_kd_tree;
  }

  double configuration_distance(const Point_d& lhs, const Point_d& rhs)
  {
	  //if (lhs == rhs) return 0;
	  //return 1;
	  Kernel_d::Vector_d v = rhs - lhs;
	  return v.squared_length();
  }

  void add_edges( const Point_d &p, int K, double EPS )
  {
	  vector<Point_d>	nearest_neighbors;


	  m_kd_tree->k_nearest_neighbors( p,
	  			K, std::back_inserter( nearest_neighbors ) );

	  for( std::vector<Point_d>::iterator q = nearest_neighbors.begin() ;
			q != nearest_neighbors.end() ; ++q )
	  {
		  //point_to_node_map_t::const_iterator  pIt = vertexID.find( p );
          //point_to_node_map_t::const_iterator  qIt = vertexID.find( *q );
		  //if(m_graph->is_in_graph(pIt->second) && m_graph->is_in_graph(qIt->second) && !m_graph->is_in_same_cc( pIt->second, qIt->second ) )
		  {
			  bool is_connect_success =  m_col->local_planner( p, *q, EPS );

			  if( is_connect_success )
			  {
				 point_to_node_map_t::const_iterator  pIt = vertexID.find( p );
				 point_to_node_map_t::const_iterator  qIt = vertexID.find( *q );

				 if( pIt != vertexID.end() && qIt != vertexID.end() )
				 {
					 int pID =  pIt->second;
					 int qID =  qIt->second;

					 double dist = configuration_distance( p, *q );//distance between q and p
					 m_graph->add_edge( pID, qID, dist );
				 }
			  }
		  }
	  }
  }

  //  This operation is supposed to generate the roadmap (sample configurations
  //  and connect them.
  void generate_roadmap()
  {
	  m_graph   = new Graph<int, Less_than_int>(0, true); // construction of an empty graph
	  //m_kd_tree = new Kd_tree_d<Kernel_d>();

	  vector<Point_d> points;

	  int num_of_samples = m_number_vertices;

	  for( int i = 0 ; i < num_of_samples ;  )
	  {
		  Point_d p = m_sampler->generate_sample();

		  if( m_col->valid_conf( p) )
		  {
			  vertexID[p] = i;
			  vertexIDToPoint[i] = p;
			  m_graph->add_vertex( i );

			  points.push_back( p );
			  i++;
		  }
	  }

	  //find nearest neighbours

	  m_kd_tree = new Kd_tree_d<Kernel_d>(); 				 // construction of an empty kd-tree
	  m_kd_tree->insert( points.begin(), points.end() ); // inserting a set of points to the structure

	  m_kd_tree->insert( m_start );
	  m_kd_tree->insert( m_target );

	  int     K      = m_k_nearest; // K nearest neighbours
	  double  EPS   =  0.1;

	  for( std::vector<Point_d>::iterator p = points.begin(); p != points.end() ; ++p )
	  {
		  add_edges( *p, K, EPS );
	  }

	  int startID  = num_of_samples + 1;
	  int targetID = num_of_samples + 2;

	  //connect start to roadmap graph
	  vertexID[m_start] = startID;
	  vertexIDToPoint[ startID ] = m_start;

	  m_graph->add_vertex( startID );
	  add_edges( m_start, K, EPS );

	  //connect target to roadmap graph
	  vertexID[m_target] = targetID;
	  vertexIDToPoint[ targetID ] = m_target;

	  m_graph->add_vertex( targetID );
	  add_edges( m_target, K, EPS );

	  is_path = m_graph->is_in_same_cc( startID, targetID );

	  if( is_path )
	  {
		  list<int> path;
		  m_graph->find_path( startID, targetID, path );

		  for( std::list<int>::iterator pID = path.begin(); pID != path.end() ; ++pID )
		  {
			  Point_d p = vertexIDToPoint[ *pID ];

			  m_path.push_back( p );
		  }
	  }
  }

  //  Returns a point path from start to target.
  //  If a path doesn't exist, returns empty vector.
  vector<Point_d> retrieve_path()
  {
	return m_path;
  }

private:
  int m_number_vertices;                // number of sampled vertices
  int m_k_nearest;                      // maximal degree of vertex in roadmap
  CollisionDetector* m_col;
  Sampler* m_sampler;
  Graph<int, Less_than_int>* m_graph;   //  Graph structure
  Kd_tree_d<Kernel_d>* m_kd_tree;         //  Kd-tree for nearest neighbor search
  Point_d m_start,m_target;             //  Start and target configurations
  point_to_node_map_t vertexID;			//  mapping point to node id
  map< int, Point_d > vertexIDToPoint;  // mapping node id to point
  bool is_path;                         // is there a path from m_start to m_target
  vector<Point_d> m_path;				// the result path
};

#endif
