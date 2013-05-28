#include "dRRT_tree_connector_t.h"

dRRT_tree_connector_t::dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler)
: m_lhs_conn_pt()
, m_rhs_conn_pt()
, m_succeeded(false)
, m_sampler(sampler)
{
	for (size_t i(0); i < p; ++i)
	{
		Point_d q_rand = m_sampler.generate_sample();
		Point_d q_0 = lhs.get_nearest(q_rand);
		Point_d q_1 = rhs.get_nearest(q_rand);
		if (local_connect(q_0,q1))
		{
			m_lhs_conn_pt = q_0;
			m_rhs_conn_pt = q_1;
			m_succeeded = true;
		}
	}
}