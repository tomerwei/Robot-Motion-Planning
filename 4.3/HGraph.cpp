#include "HGraph.h"
#include <cassert>

HGraph::HGraph(const Point_d& start_pos, const Point_d& end_pos, const distance_metric& dm)
:m_graph()
,m_vertices()
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

	double max_distance = 0;
	for(int i = 1; i < path.size(); ++i)
	{
		const Point_d& prev(path.at(i-1));
		const Point_d& curr(path.at(i));

		if(!m_graph.is_in_graph(curr))
			m_graph.add_vertex(curr);

		double weight = m_distance_metric(prev,curr);
		if (weight > max_distance)
			max_distance = weight;
		m_graph.add_edge(prev,curr,weight);
	}

	for(point_set_t::const_iterator it = m_vertices.begin(), it_end = m_vertices.end(); it != it_end; ++it)
	{
		for(vector<Point_d>::const_iterator p = path.begin(), p_end = path.end(); p != p_end; ++p)
		{
			double d = m_distance_metric(*it,*p);
			if (d < max_distance)
			{
				//if collision not detected...
				m_graph.add_edge(*it,*p,d);
			}
		}
	}

	m_vertices.insert(m_path.begin(),m_path.end());
	m_graph.find_path(m_start,m_target,m_path);
}

const list<Point_d>& HGraph::get_path() const
{
	return m_path;
}