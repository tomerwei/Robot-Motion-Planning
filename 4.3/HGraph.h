#ifndef __HGRAPH_INCLUDED__
#define __HGRAPH_INCLUDED__
#include "Graph.h"
#include "basic_typedef.h"

class HGraph
{
public:
	HGraph(const Point_d& start_pos, const Point_d& end_pos);
	~HGraph() {}

	void push_back(const vector<Point_d>& path);
	const vector<Point_d>& get_path() const;

private:
	Graph<Point_d, point_d_less> m_graph;
	double (*m_distance_metric)(const Point_d&, const Point_d&);
};

#endif //__HGRAPH_INCLUDED__