#ifndef __DRRT_TREE_CONNECTTOR_T_H__
#define __DRRT_TREE_CONNECTTOR_T_H__
#include "RTT_tree_t.h"
#include "Sampler.h"


class dRRT_tree_connector_t
{
public:
	dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler);
	bool is_connected() const {return m_succeeded;}
	const Point_d &lhs_conn_pt() const {return m_lhs_conn_pt; }
	const Point_d &rhs_conn_pt() const {return m_rhs_conn_pt; }
private:
	Point_d m_lhs_conn_pt;
	Point_d m_rhs_conn_pt;
	bool m_succeeded;
	const Sampler &m_sampler;
};

#endif //__DRRT_TREE_CONNECTTOR_T_H__