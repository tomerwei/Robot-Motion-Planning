#ifndef __HGRAPH_INCLUDED__
#define __HGRAPH_INCLUDED__
#include "Graph.h"
#include "CollisionDetector.h"
#include "basic_typedef.h"
#include <set>

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
	HGraph(const Point_d& start_pos, const Point_d& end_pos, const distance_metric& dm, const LocalPlanner &lp);
	~HGraph() {}

	void push_back(const vector<Point_d>& path);
	const list<Point_d>& get_path() const;
	double get_distance() const;

private:
	typedef std::set<Point_d, point_d_less> point_set_t;
	Graph<Point_d, point_d_less> m_graph;
	point_set_t m_vertices;
	const distance_metric &m_distance_metric;
	Point_d m_start,m_target;
	const LocalPlanner& m_lp;
	std::list<Point_d> m_path;
	double m_distance;
};

#endif //__HGRAPH_INCLUDED__