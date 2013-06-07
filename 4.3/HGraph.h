#ifndef __HGRAPH_INCLUDED__
#define __HGRAPH_INCLUDED__
#include "Graph.h"
#include "basic_typedef.h"

class HGraph
{
public:
	HGraph();
	~HGraph() {}

	void push_back(const vector<Point_d>& path);
	const vector<Point_d>& get_path() const;

private:
	Graph<Point_d, point_d_less> m_graph;
};

#endif //__HGRAPH_INCLUDED__