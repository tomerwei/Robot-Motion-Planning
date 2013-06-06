#ifndef CLEARANCE_IE_H
#define CLEARANCE_IE_H

#include "basic_typedef.h"
#include <CGAL/minkowski_sum_2.h>
#include "Kd_tree.h"

class Clearance_detector_ie
{
public:
	
	////////////////////////
	// CTORS
	////////////////////////
	Clearance_detector_ie(	vector<Polygon_2> *obstacles, 
						Polygon_2 robot, 
						double eps) :	m_obstacles(obstacles), 
										m_robot(robot),
										m_eps(eps)
	{
		// generate the arrangment
		generate_arrangement();
		generate_clearance_structure();
	}

	void generate_arrangement()
	{
		
		Polygon_set_2 set;
		Polygon_2 mirrored_robot = mirror_robot();
		turn_counterclockwise(mirrored_robot);

		for (vector<Polygon_2>::iterator iter = m_obstacles->begin();
				iter != m_obstacles->end(); iter++)
		{
			Polygon_2 obs = *iter;
			turn_counterclockwise(obs);

			Polygon_with_holes_2	poly_wh = CGAL::minkowski_sum_2(obs, 
												mirrored_robot);
			set.join(poly_wh);
		}

		m_arr = new Arrangement_2(set.arrangement());
	}

	void generate_clearance_structure()
	{
		m_kd_tree = new Kd_tree_2<Kernel_ie>();
		vector<Point_2_ie>	points;
		int num = m_arr->number_of_vertices();
		for (Arrangement_2::Vertex_iterator vit = m_arr->vertices_begin(); 
				vit != m_arr->vertices_end(); ++vit)
		{
			if (vit->is_at_open_boundary())
				continue;
			Point_2 p = vit->point();
			points.push_back(to_ie(p));
		}

		for (Arrangement_2::Edge_iterator	eit = m_arr->edges_begin(); 
				eit	!= m_arr->edges_end(); ++eit)
		{
			Point_2 p_source(eit->source()->point()), 
				p_target(eit->target()->point());

			Vector_2 vec(p_source, p_target);

			double dist = distance(p_source, p_target);

			int step_num = ceil(dist / m_eps);
			for (int i = 1; i < step_num; ++i)
			{
				double ratio = ((double)i / step_num);
				Point_2 p_in_step = p_source + ratio * vec;
				points.push_back(to_ie(p_in_step));
			}
		}

		for (int i = 0; i < points.size(); i++)
		{
			//std::cout << "( " << CGAL::to_double(points[i].x()) << ", " << CGAL::to_double(points[i].y()) << ")\n";
		}

		m_kd_tree->insert(points.begin(), points.end()); 
	}

	double clearance(Point_2_ie point)
	{
		Point_2_ie p_nearest = m_kd_tree->nearest_neighbor(point);
		return distance(p_nearest, point);
	}

	Polygon_2 mirror_robot()
	{
		Polygon_2 result;
		for (int i = 0; i < m_robot.size(); i++)
		{
			Point_2 poly_point =m_robot[i];
			Point_2 result_point(- poly_point.x(), - poly_point.y());
			result.push_back(result_point);
		}
		return result;
	}

	double			distance(Conf s, Conf t)
	{
		return distance(to_ie(s),to_ie(t));
	}

	double			distance(Point_2_ie s, Point_2_ie t)
	{
		double x1 = s.x();
		double y1 = s.y() ;
		double x2 = t.x() ;
		double y2 = t.y() ;

		double dist = sqrt(pow(x1 - x2, 2) + pow(y1 - y2,2));
		return dist;
	}

	Point_2_ie	to_ie(Point_2 p)
	{
		return Point_2_ie(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
	}

	void			turn_counterclockwise(Polygon_2& p)
	{
		if (p.is_clockwise_oriented())
			p.reverse_orientation();
	}

	////////////////////////
	// Data Members
	////////////////////////
	//Ric_pl*								m_pl;
	Arrangement_2*						m_arr;
	vector<Polygon_2>*					m_obstacles;
	Polygon_2							m_robot;
	double								m_eps;
	Kd_tree_2<Kernel_ie>*					m_kd_tree;
};

#endif // CLEARANCE_H