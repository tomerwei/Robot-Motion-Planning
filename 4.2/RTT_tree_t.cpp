#include "RTT_tree_t.h"


RRT_tree_t::RRT_tree_t(const std::vector<Conf>& tree_root, const Sampler &sampler)
: m_root(tree_root)
, m_knn_container()
, m_tree()
, m_sampler(sampler)
{
}

void RRT_tree_t::expand(size_t samples)
{
	for(size_t i(0); i < samples; ++i)
	{
		Point_d q_rand = m_sampler.generate_sample();
		Point_d q_near = m_knn_container.nearest_neighbor(q_rand);
		Point_d q_new = something_direction_oracle(q_rand,q_near);

		if (???)
		{
			m_tree.add_vertex(q_new);
			m_tree.add_edge(q_near,q_new);
		}
	}
}

const Point_d RRT_tree_t::get_nearest(const Point_d& nearest_to) const
{
	return m_knn_container.nearest_neighbor(nearest_to);
}