#ifndef __RTT_TREE_T_H_DEFINED__
#define __RTT_TREE_T_H_DEFINED__


#include "Kd_tree_d.h"
#include "Graph.h"
#include "Sampler.h"
#include "Prm.h"
#include "basic_typedef.h"
#include <vector>

class RRT_tree_t
{
public:
	RRT_tree_t(const std::vector<Conf>& tree_root, const Prm& r1_roadmap, const Prm& r2_roadmap);
	void expand(size_t samples);
	const Point_d get_nearest(const Point_d& nearest_to) const;

private:
	Vector_2 make_random_direction_vec();
	Point_d new_from_direction_oracle(const Point_d &pt);
private:
	std::vector<Conf> m_root;
	Kd_tree_d<Kernel_d> m_knn_container;
	Graph<Kernel_d> m_tree;
	const Prm &m_r1_roadmap;
	const Prm &m_r2_roadmap;
};

#endif //__RTT_TREE_T_H_DEFINED__