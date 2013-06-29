#include "SRPrm.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>

SRPrm::SRPrm(
	int number_vertices, 
	int k_nearest, double epsilon,
	const SRCollisionDetector& col, 
	const SRSampler &sampler,
	Point_2 start, 
	Point_2 target
) 
: m_number_vertices(number_vertices)
, m_k_nearest(k_nearest)
, m_col(col)
, m_sampler(sampler)
, m_graph(0)
, m_start(start)
, m_target(target)
, m_epsilon(epsilon)
{
}

SRPrm::~SRPrm()
{
	delete m_graph;
	delete m_kd_tree;
}

void SRPrm::add_edges( Point_2 p, int K, double EPS )
{
	vector<Point_2>	nearest_neighbors;

	m_kd_tree->k_nearest_neighbors( p,
	  		K, std::back_inserter( nearest_neighbors ) );

	for( std::vector<Point_2>::iterator q = nearest_neighbors.begin() ;
		q != nearest_neighbors.end() ; ++q )
	{
		bool is_connect_success =  m_col.local_planner( p, *q, EPS );

		if( is_connect_success )
		{
			std::map<Point_2,int>::iterator  pIt = vertexID.find( p );
			std::map<Point_2,int>::iterator  qIt = vertexID.find( *q );

			if( pIt != vertexID.end() && qIt != vertexID.end() )
			{
				int pID =  pIt->second;
				int qID =  qIt->second;

				double dist = CGAL::to_double(CGAL::squared_distance( p, *q ));//distance between q and p
				m_graph->add_edge( pID, qID, dist );
			}
		}
	}
}

void SRPrm::generate_roadmap()
{
	m_graph   = new Graph<int, Less_than_int>(0, true); // construction of an empty graph

	vector<Point_2> points;

	int num_of_samples = m_number_vertices;

	for( int i = 0 ; i < num_of_samples ;  )
	{
		Point_2 p = m_sampler.generate_sample();

		if( (vertexID.find(p) == vertexID.end()) && m_col.valid_conf( p) )
		{
			vertexID[p] = i;
			vertexIDToPoint[i] = p;
			m_graph->add_vertex( i );

			points.push_back( p );
			i++;
		}
	}

	//find nearest neighbours
	m_kd_tree = new Kd_tree_2<Kernel>(); 				 // construction of an empty kd-tree
	m_kd_tree->insert( points.begin(), points.end() ); // inserting a set of points to the structure
	m_kd_tree->insert( m_start );
	m_kd_tree->insert( m_target );

	int     K      = m_k_nearest; // K nearest neighbours

	for( std::vector<Point_2>::iterator p = points.begin(); p != points.end() ; ++p )
	{
		add_edges( *p, K, m_epsilon );
	}

	int startID  = num_of_samples + 1;
	int targetID = num_of_samples + 2;

	//connect start to roadmap graph
	vertexID[m_start] = startID;
	vertexIDToPoint[ startID ] = m_start;

	m_graph->add_vertex( startID );
	add_edges( m_start, K, m_epsilon );

	//connect target to roadmap graph
	vertexID[m_target] = targetID;
	vertexIDToPoint[ targetID ] = m_target;

	m_graph->add_vertex( targetID );
	add_edges( m_target, K, m_epsilon );

	is_path = m_graph->is_in_same_cc( startID, targetID );

	if( is_path )
	{
		list<int> path;
		m_graph->find_path( startID, targetID, path );

		for( std::list<int>::iterator pID = path.begin(); pID != path.end() ; ++pID )
		{
			Point_2 p = vertexIDToPoint[ *pID ];

			m_path.push_back( p );
		}
	}
}

void SRPrm::get_neighbors(const Point_2& pt, std::back_insert_iterator<std::vector<Point_2> > out) const
{
	std::map<Point_2,int>::const_iterator  pIt = vertexID.find( pt );
	std::vector<int> pt_ids;
	m_graph->get_neighbors(pIt->second,std::back_inserter(pt_ids));

	BOOST_FOREACH(int id,pt_ids)
	{
		*++out = vertexIDToPoint.find(id)->second;
	}
}

Point_2 SRPrm::get_nearest_to(const Point_2 &pt) const
{
	return this->m_kd_tree->nearest_neighbor(pt);
}