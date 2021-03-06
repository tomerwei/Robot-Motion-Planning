#include "HGraph.h"
#include <cassert>

HGraph::HGraph(const Point_d& start_pos, const Point_d& end_pos, const distance_metric& dm, const LocalPlanner& lp)
:m_graph()
,m_vertices()
,m_distance_metric(dm)
,m_start(start_pos)
,m_target(end_pos)
,m_lp(lp)
,m_path()
,m_distance()
{
	m_graph.add_vertex(start_pos);
	m_graph.add_vertex(end_pos);
}

void HGraph::push_back(const vector<Point_d>& path)
{
	assert(m_graph.is_in_graph(path[0]) && m_graph.is_in_graph(path[path.size() - 1]));

	double max_distance = 0;
	double path_sum = 0;
	for(int i = 1; i < path.size(); ++i)
	{
		const Point_d& prev(path.at(i-1));
		const Point_d& curr(path.at(i));

		if(!m_graph.is_in_graph(curr))
			m_graph.add_vertex(curr);

		double weight = m_distance_metric(prev,curr);
		path_sum += weight;
		if (weight > max_distance)
			max_distance = weight;
		m_graph.add_edge(prev,curr,weight);
	}

	for(point_set_t::const_iterator it = m_vertices.begin(), it_end = m_vertices.end(); it != it_end; ++it)
	{
		for(vector<Point_d>::const_iterator p = path.begin(), p_end = path.end(); p != p_end; ++p)
		{
			if (*it != *p) {
				double d = m_distance_metric(*it,*p);
				if (d < max_distance)
				{
					if (m_lp.local_planner(*it,*p)) //no collision on the way
						m_graph.add_edge(*it,*p,d);
				}
			}
		}
	}

	m_vertices.insert(path.begin(),path.end());
	m_path.resize(0);
	m_graph.find_weighted_path(m_start,m_target,m_path,m_distance);
	assert(m_path.front() == m_start);
	assert(m_path.back() == m_target);
}

const list<Point_d>& HGraph::get_path() const
{
	return m_path;
}

double HGraph::get_distance() const
{
	return m_distance;
}