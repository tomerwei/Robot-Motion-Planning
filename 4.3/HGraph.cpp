#include "HGraph.h"
#include <cassert>

namespace {
	double path_length_dmetric(const Point_d& lhs, const Point_d& rhs)
	{
		const Point_2 r1_lhs(lhs.cartesian(0),lhs.cartesian(1)),
			r2_lhs(lhs.cartesian(2),lhs.cartesian(3)),
			r1_rhs(rhs.cartesian(0),rhs.cartesian(1)),
			r2_rhs(rhs.cartesian(2),rhs.cartesian(3));

		double r1_sq = CGAL::to_double(CGAL::squared_distance(r1_lhs,r1_rhs)),
			r2_sq = CGAL::to_double(CGAL::squared_distance(r2_lhs,r2_rhs));
		return sqrt(r1_sq) + sqrt(r2_sq);
	}
}

HGraph::HGraph(const Point_d& start_pos, const Point_d& end_pos, const distance_metric& dm)
:m_graph()
,m_distance_metric(dm)
,m_start(start_pos)
,m_target(end_pos)
,m_path()
{
	m_graph.add_vertex(start_pos);
	m_graph.add_vertex(end_pos);
}

void HGraph::push_back(const vector<Point_d>& path)
{
	assert(m_graph.is_in_graph(path[0]) && m_graph.is_in_graph(path[path.size() - 1]));

	for(int i = 1; i < path.size(); ++i)
	{
		const Point_d& prev(path.at(i-1));
		const Point_d& curr(path.at(i));

		if(!m_graph.is_in_graph(curr))
			m_graph.add_vertex(curr);

		double weight = m_distance_metric(prev,curr);
		m_graph.add_edge(prev,curr,weight);
	}

	m_graph.find_path(m_start,m_target,m_path);
}

const list<Point_d>& HGraph::get_path() const
{
	return m_path;
}