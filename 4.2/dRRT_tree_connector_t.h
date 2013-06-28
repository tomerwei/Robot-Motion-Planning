#ifndef __DRRT_TREE_CONNECTTOR_T_H__
#define __DRRT_TREE_CONNECTTOR_T_H__
#include "RTT_tree_t.h"
#include "Sampler.h"
#include <CGAL/Cartesian_d.h>
#include <basic_typedef.h>


class dRRT_tree_connector_t
{
public:
	dRRT_tree_connector_t(const RRT_tree_t& lhs, const RRT_tree_t& rhs, const Sampler &sampler, const LocalPlanner& lp, size_t p);
	bool is_connected() const {return m_succeeded;}
	const list<Point_d>& get_path() const;
	const Point_d &lhs_conn_pt() const {return m_lhs_conn_pt; }
	const Point_d &rhs_conn_pt() const {return m_rhs_conn_pt; }
private:
	bool local_connect(const Point_d& p1, const Point_d& p2);
private:
	Point_d m_lhs_conn_pt;
	Point_d m_rhs_conn_pt;
	bool m_succeeded;
	const Sampler &m_sampler;
	const LocalPlanner& m_lp;
	std::list<Point_d> m_path;
};

#endif //__DRRT_TREE_CONNECTTOR_T_H__
