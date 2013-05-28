#ifndef __RTT_TREE_T_H_DEFINED__
#define __RTT_TREE_T_H_DEFINED__


#include "Kd_tree_d.h"
#include "Graph.h"
#include "Sampler.h"
#include "basic_typedef.h"
#include <vector>

class RRT_tree_t
{
public:
	RRT_tree_t(const std::vector<Conf>& tree_root, const Sampler &sampler);
	void expand(size_t samples);

private:
	std::vector<Conf> m_root;
	Kd_tree_d<Kernel_d> m_knn_container;
	Graph<Kernel_d> m_tree;
	const Sampler &m_sampler;
};

#endif //__RTT_TREE_T_H_DEFINED__