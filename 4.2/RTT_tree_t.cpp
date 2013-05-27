#include "RTT_tree_t.h"


RRT_tree_t::RRT_tree_t(const std::vector<Conf>& tree_root)
: m_root(tree_root)
, m_knn_container()
, m_tree()
{
}

void RRT_tree_t::expand(size_t samples)
{
	for(size_t i(0); i < samples; ++i)
	{
		Point_d q_rand = random_sample();
		Point_d q_near = m_knn_container.nearest_neighbor(q_rand);
		Point_d q_new = something_direction_oracle(q_rand,q_near);

		if (???)
		{
			m_tree.add_vertex(q_new);
			m_tree.add_edge(q_near,q_new);
		}
	}
}