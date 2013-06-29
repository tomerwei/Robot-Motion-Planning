#ifndef __RTT_TREE_T_H_DEFINED__
#define __RTT_TREE_T_H_DEFINED__


#include "Kd_tree_d.h"
#include "Graph.h"
#include "Sampler.h"
#include "SRPrm.h"
#include "CollisionDetector.h"
#include "basic_typedef.h"
#include <vector>
#include <map>
#include <utility>


class RRT_tree_t
{
	typedef map< Point_d, double, point_d_less > point_to_cost_map_t;


public:
	RRT_tree_t(
		const std::vector<Conf>& tree_root, 
		const SRPrm& r1_roadmap, 
		const SRPrm& r2_roadmap, 
		const LocalPlanner& local_planner, 
		const Sampler& sampler
	);
	void expand(size_t samples);
	void get_neighbors(  Point_d& pt, std::back_insert_iterator<std::vector<Point_d> > it) ;
	const Point_d get_nearest(const Point_d& nearest_to) const;
	Point_d get_root();

private:
	Vector_2 make_random_direction_vec();
	Point_d new_from_direction_oracle(const Point_d &pt, const Point_d& dir_of);
	Point_d virtual_graph_nearest_neighbor(const Point_d &pt);
	Point_d to_pointd(const Point_2& r1, const Point_2& r2);
private:
 
	std::vector<Conf> m_root;
	mutable Kd_tree_d<Kernel_d> m_knn_container;
	Graph<int, Less_than_int> m_tree;
	const SRPrm &m_r1_roadmap;
	const SRPrm &m_r2_roadmap;
	const LocalPlanner& m_local_planner;
	std::vector<double> m_cnv;
	const Sampler& m_sampler;
	mutable std::vector<Point_d> m_knn_out;

	Point_d m_tree_root;
	point_to_cost_map_t vertexID;			//  mapping point to node id
	map< int, Point_d > vertexIDToPoint;  // mapping node id to point
};

#endif //__RTT_TREE_T_H_DEFINED__
