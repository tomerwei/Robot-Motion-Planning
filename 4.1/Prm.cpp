#include "Prm.h"

Prm::Prm(int number_vertices, 
      int k_nearest, 
      const CollisionDetector& col, 
      const Sampler& sampler,
      Point_d start, Point_d target)
:m_number_vertices(number_vertices)              // number of sampled vertices
,m_k_nearest(k_nearest)                      // maximal degree of vertex in roadmap
,m_col(col)
,m_sampler(sampler)
,m_loc_planner(m_col)
,m_graph(NULL)   //  Graph structure
,m_kd_tree()         //  Kd-tree for nearest neighbor search
,m_start(start)
,m_target(target)             //  Start and target configurations
,vertexID()			//  mapping point to node id
,vertexIDToPoint()  // mapping node id to point
,is_path(false)                         // is there a path from m_start to m_target
,m_path()				// the result path
{
}

double Prm::configuration_distance(const Point_d& lhs, const Point_d& rhs)
{
	//if (lhs == rhs) return 0;
	//return 1;
	Kernel_d::Vector_d v = rhs - lhs;
	return v.squared_length();
}

void Prm::add_edges( const Point_d &p, int K, double EPS )
{
	vector<Point_d>	nearest_neighbors;


	m_kd_tree.k_nearest_neighbors( p,
	  		K, std::back_inserter( nearest_neighbors ) );

	for( std::vector<Point_d>::iterator q = nearest_neighbors.begin() ;
		q != nearest_neighbors.end() ; ++q )
	{
		//point_to_node_map_t::const_iterator  pIt = vertexID.find( p );
        //point_to_node_map_t::const_iterator  qIt = vertexID.find( *q );
		//if(m_graph->is_in_graph(pIt->second) && m_graph->is_in_graph(qIt->second) && !m_graph->is_in_same_cc( pIt->second, qIt->second ) )
		{

			bool is_connect_success =  m_loc_planner.local_planner( p, *q, EPS );

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

void Prm::generate_roadmap()
{
	  m_graph   = new Graph<int, Less_than_int>(0, true); // construction of an empty graph
	  //m_kd_tree = new Kd_tree_d<Kernel_d>();

	  vector<Point_d> points;

	  int num_of_samples = m_number_vertices;

	  for( int i = 0 ; i < num_of_samples ;  )
	  {
		  Point_d p = m_sampler.generate_sample();

		  if( m_col.valid_conf( p) )
		  {
			  vertexID[p] = i;
			  vertexIDToPoint[i] = p;
			  m_graph->add_vertex( i );

			  points.push_back( p );
			  i++;
		  }
	  }

	  //find nearest neighbours

	  m_kd_tree.insert( points.begin(), points.end() ); // inserting a set of points to the structure

	  m_kd_tree.insert( m_start );
	  m_kd_tree.insert( m_target );

	  int     K      = m_k_nearest; // K nearest neighbours
	  double  EPS   =  4.5;        //TODO change

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
