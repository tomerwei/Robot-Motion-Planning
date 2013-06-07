#ifndef __HGRAPH_INCLUDED__
#define __HGRAPH_INCLUDED__
#include "Graph.h"
#include "basic_typedef.h"

class HGraph
{
public:
	struct distance_metric
	{
		double operator()(const Point_d& lhs, const Point_d& rhs) const { return do_measure(lhs,rhs); }
	private:
		virtual double do_measure(const Point_d&, const Point_d&) const = 0;
	};

public:
	HGraph(const Point_d& start_pos, const Point_d& end_pos, const distance_metric& dm);
	~HGraph() {}

	void push_back(const vector<Point_d>& path);
	const list<Point_d>& get_path() const;

private:
	Graph<Point_d, point_d_less> m_graph;
	const distance_metric &m_distance_metric;
	Point_d m_start,m_target;
	std::list<Point_d> m_path;
};

#endif //__HGRAPH_INCLUDED__