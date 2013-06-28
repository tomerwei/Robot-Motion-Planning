#include "dRRT_tree_connector_t.h"



dRRT_tree_connector_t::dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler, const LocalPlanner& lp, size_t p)
: m_lhs_conn_pt()
, m_rhs_conn_pt()
, m_succeeded(false)
, m_sampler(sampler)
, m_lp(lp)
, m_path()
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

const list<Point_d>& dRRT_tree_connector_t::get_path() const
{
	//calculate m_path via the A* algorithm
	//http://en.wikipedia.org/wiki/IDA*
	//http://en.wikipedia.org/wiki/Fringe_search

	return m_path;
}



double configuration_distance(Point_d lhs, Point_d rhs)
{
	//if (lhs == rhs) return 0;
	//return 1;
	Kernel_d::Vector_d v = rhs - lhs;
	return v.squared_length();
}

double heuristic_cost_estimate( Point_d cur_node, Point_d goal )
{//straight line distance
	return configuration_distance( cur_node, goal );
}


double ida_star_cost( Point_d goal, Point_d node, double cost_so_far )
{
	double g   = cost_so_far; //cost from root to node.
	double h   = heuristic_cost_estimate( node, goal );
	double res = -1;

	if( g >= 0 && h >= 0 )
	{
		res = g + h;
	}

	return res;
}


bool ida_star_is_node_goal( Point_d node, Point_d goal )
{
	return node == goal;
}


vector <Point_d>  ida_star_successors_get( Point_d node )
{

	return vector<Point_d >(NULL);
}



//lets do: root = m_lhs or rhs_conn
//goal: tree root
vector<Point_d >  ida_star_depth_limited_search( Point_d start, Point_d goal,
		                               double cost_so_far, // Cost from start along best known path.
		                               vector<Point_d > &path_so_far,
		                               double *cost_limit )
{
	int path_len         = path_so_far.size();
	Point_d node         = path_so_far[path_len-1];
    double minimum_cost = ida_star_cost( goal, node, cost_so_far );

    if( minimum_cost > *cost_limit)
    {
    	*cost_limit = minimum_cost;
    	return vector<Point_d >( NULL );
    }

    if( ida_star_is_node_goal( node, goal ) )
    {
    	//  	*cost_limit = -1;
        return path_so_far;
    }

    double next_cost_limit = -1;  //INIFINITE
    vector   < vector < Point_d > > all_solutions;
    vector   < double > all_solutions_cost;

    vector <Point_d > successors = ida_star_successors_get( node );

    //for( Point_d s :  successors ) //successors = nearest neighbors of the node
    for( vector<Point_d>::const_iterator s = successors.begin() ; s != successors.end() ; s++ )
    {
        double new_start_cost = cost_so_far + configuration_distance( node , *s );
        double new_cost_limit = -1;

        vector < Point_d > solution(path_so_far);
        solution.push_back( *s );

        solution = ida_star_depth_limited_search( start, goal, new_start_cost, solution, &new_cost_limit );

        if( solution != ( vector<Point_d > (NULL) ) )
        {
        	all_solutions.push_back( solution );
        	all_solutions_cost.push_back( new_cost_limit );
        }

        if( new_cost_limit > 0 )
        {
        	next_cost_limit = new_cost_limit;
        }
    }

    if( all_solutions.size() > 0 )
    {

    	int   tmp_cost_limit = -1;
    	int   pos             =  0;

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

vector <Point_d> ida_algorithm( Point_d start, Point_d goal, int cost_limit )
{

	for( int i = 0 ; i < 500 ; ++i )
	{
		vector     < Point_d >  start_list;
		vector     < Point_d >  solution;
		double   m_cost_limit   = cost_limit;

		start_list.push_back(start );

		solution = ida_star_depth_limited_search( start, goal, 0.0, start_list, &m_cost_limit );

        if( solution != ( vector <Point_d>(NULL)) )
        {
            return solution;
        }

	}
	return vector <Point_d>( NULL );
}


