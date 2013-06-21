#include "dRRT_tree_connector_t.h"

dRRT_tree_connector_t::dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler, const LocalPlanner& lp, size_t p)
: m_lhs_conn_pt()
, m_rhs_conn_pt()
, m_succeeded(false)
, m_sampler(sampler)
, m_lp(lp)
{
	for (size_t i(0); i < p; ++i)
	{
		Point_d q_rand = m_sampler.generate_sample();
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

//bool dRRT_tree_connector_t::local_connect(const Point_d& p1, const Point_d& p2)
//{
//}