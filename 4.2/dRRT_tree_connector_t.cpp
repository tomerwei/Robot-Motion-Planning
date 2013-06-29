#include "dRRT_tree_connector_t.h"


#define INFINITE    999999.0
#define K_NOT_USED  -17.0


dRRT_tree_connector_t::dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler, const LocalPlanner& lp, size_t p)
: m_lhs_conn_pt()
, m_rhs_conn_pt()
, m_succeeded(false)
, m_sampler(sampler)
, m_lp(lp)
, m_path()
, node_cost()
{
	for (size_t i(0); i < p; ++i)
	{
		Point_d q_rand = m_sampler.generate_sample_no_obstacles();
		const Point_d 
			q_0 = lhs.get_nearest(q_rand),
			q_1 = rhs.get_nearest(q_rand);
		if (local_connect(q_0,q_1))
		{
			m_lhs_conn_pt = q_0;
			m_rhs_conn_pt = q_1;
			m_succeeded = true;
		}
	}
}

bool dRRT_tree_connector_t::local_connect(const Point_d& p1, const Point_d& p2)
{
	return this->m_lp.local_planner(p1,p2);
}

double configuration_distance(Point_d lhs, Point_d rhs)
{
	Kernel_d::Vector_d v = rhs - lhs;
	return sqrt( v.squared_length() );
}

double heuristic_cost_estimate( Point_d cur_node, Point_d goal )
{
	//straight line distance
	return configuration_distance( cur_node, goal );
}


double ida_star_cost( Point_d goal, Point_d node, double cost_so_far )
{
	double g   = cost_so_far; //cost from root to node.
	double h   = heuristic_cost_estimate( node, goal );
	double res = 0.0;

	res = g + h;

	return res;
}


vector <Point_d>  ida_star_successors_get( RRT_tree_t& tree, Point_d node )
{
	std::vector<Point_d> nn;
	nn.resize(0);
	tree.get_neighbors( node , std::back_inserter(nn) );
	return nn;
}


vector<Point_d >  dRRT_tree_connector_t::ida_star_depth_limited_search(
										RRT_tree_t& tree,
										Point_d start, Point_d goal,
		                               double cost_so_far, // Cost from start along best known path.
		                               vector<Point_d > &path_so_far,
		                               double *cost_limit )
{
	int     path_len      = path_so_far.size();
	Point_d  node         = path_so_far[path_len-1];
    double  minimum_cost = ida_star_cost( goal, node, cost_so_far );
    bool    to_abort      = false;


	point_to_cost_map_t::const_iterator iter = node_cost.find(node);

    if( iter != node_cost.end() )
    {

    	double old_cost = iter->second;

    	if( minimum_cost > old_cost )
    	{
    		to_abort = true;
    	}
    	else
    	{
    		node_cost[node] = minimum_cost;
    	}
    }
    else
    {
    	node_cost[node] = minimum_cost;
    }

    if( to_abort ||  minimum_cost > *cost_limit )
    {
    	*cost_limit = minimum_cost;
    	return vector<Point_d >( NULL );
    }

    if( node == goal )
    {
        return path_so_far;
    }

    double next_cost_limit = INFINITE;
    vector   < vector < Point_d > > all_solutions;
    vector   < double > all_solutions_cost;

    vector <Point_d > successors = ida_star_successors_get( tree, node );

    for( vector<Point_d>::const_iterator s = successors.begin() ; s != successors.end() ; s++ )
    {

    	if( *s != node )
    	{
    		double new_start_cost = cost_so_far + configuration_distance( node , *s );
    		double new_cost_limit = INFINITE;

    		vector < Point_d > solution(path_so_far);
    		solution.push_back( *s );

    		solution = ida_star_depth_limited_search( tree, start, goal, new_start_cost, solution, &new_cost_limit );

    		if( solution != ( vector<Point_d > (NULL) ) )
    		{
    			all_solutions.push_back( solution );
    			all_solutions_cost.push_back( new_cost_limit );
    		}

    		next_cost_limit = min( next_cost_limit, new_cost_limit );
    	}
    }

    if( all_solutions.size() > 0 )
    {

    	double   tmp_cost_limit  =  K_NOT_USED;
    	int       pos             =  0;

    	for( int i = 0; i < all_solutions_cost.size(); ++i )
    	{

    		if( all_solutions_cost[i] > 0 )
    		{

    			if( tmp_cost_limit > 0 )
    			{
    				if( all_solutions_cost[i] > tmp_cost_limit )
    				{
    					tmp_cost_limit = all_solutions_cost[i];
    					pos = i;
    				}
    			}
    			else
    			{
    				tmp_cost_limit = all_solutions_cost[i];
    				pos = i;
    			}
    		}
    	}

    	vector <Point_d > sol( all_solutions[pos] );
    	*cost_limit = tmp_cost_limit;
    	return sol;
    }

    *cost_limit  = next_cost_limit;
    return vector<Point_d >(NULL);
}

vector <Point_d> dRRT_tree_connector_t::ida_algorithm( RRT_tree_t& tree,
		                                                    Point_d start,
		                                                    Point_d goal )
{
	vector     < Point_d >  start_list;
	vector     < Point_d >  solution;
	double   m_cost_limit   =  INFINITE;

	node_cost.clear();
	start_list.resize(0);
	solution.resize(0);

	start_list.push_back(start );
	solution = ida_star_depth_limited_search( tree, start, goal, 0.0, start_list, &m_cost_limit );

	return solution;
}

vector<vector<Conf> > dRRT_tree_connector_t::get_path( RRT_tree_t& tree1, Point_d connect_pnt1,
													  RRT_tree_t& tree2, Point_d connect_pnt2 )
{
	vector < Point_d > path1;
	vector < Point_d > path2;
	vector<vector<Conf> > result;

	Point_d root1( tree1.get_root() );
	path1 = ida_algorithm( tree1, connect_pnt1 , root1 );

	Point_d root2( tree2.get_root() );
	path2 = ida_algorithm( tree2, connect_pnt2, root2 );

	for( vector<Point_d>::reverse_iterator s = path1.rbegin() ; s != path1.rend() ; s++ )
	{
		vector<Conf> tmp;
		Point_2 robo1_pos( s->cartesian(0),s->cartesian(1) );
		Point_2 robo2_pos( s->cartesian(2),s->cartesian(3) );
		tmp.push_back( robo1_pos );
		tmp.push_back( robo2_pos );

		result.push_back( tmp );
	}

	for( vector<Point_d>::const_iterator s = path2.begin() ; s != path2.end() ; s++ )
	{
		vector<Conf> tmp;
		Point_2 robo1_pos( s->cartesian(0),s->cartesian(1) );
		Point_2 robo2_pos( s->cartesian(2),s->cartesian(3) );
		tmp.push_back( robo1_pos );
		tmp.push_back( robo2_pos );

		result.push_back( tmp );
	}

	return result;
}

