#ifndef __DRRT_TREE_CONNECTTOR_T_H__
#define __DRRT_TREE_CONNECTTOR_T_H__
#include "RTT_tree_t.h"
#include "Sampler.h"
#include <CGAL/Cartesian_d.h>
#include <basic_typedef.h>
#include <map>
#include <utility>


using namespace std;


class dRRT_tree_connector_t
{
	typedef map< Point_d, double, point_d_less > point_to_cost_map_t;

public:
	dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler, const LocalPlanner& lp, size_t p);
	void get_path(  RRT_tree_t& tree1, Point_d pnt1 );
	bool is_connected() const {return m_succeeded;}
	const Point_d &lhs_conn_pt() const {return m_lhs_conn_pt; }
	const Point_d &rhs_conn_pt() const {return m_rhs_conn_pt; }
private:
	bool local_connect(const Point_d& p1, const Point_d& p2);
	vector <Point_d > ida_algorithm( RRT_tree_t& tree, Point_d start, Point_d goal );
	vector <Point_d >  ida_star_depth_limited_search(
											RRT_tree_t& tree,
											Point_d start, Point_d goal,
			                               double cost_so_far, // Cost from start along best known path.
			                               vector<Point_d > &path_so_far,
			                               double *cost_limit );
private:
	Point_d m_lhs_conn_pt;
	Point_d m_rhs_conn_pt;
	bool m_succeeded;
	const Sampler &m_sampler;
	const LocalPlanner& m_lp;
	std::list<Point_d > m_path;
	point_to_cost_map_t node_cost;
};

#endif //__DRRT_TREE_CONNECTTOR_T_H__
