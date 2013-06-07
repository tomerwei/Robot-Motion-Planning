#include "HGraph.h"
#include <cassert>

HGraph::HGraph(const Point_d& start_pos, const Point_d& end_pos)
:m_graph()
,m_distance_metric()
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
}