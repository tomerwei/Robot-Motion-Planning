#include "CollisionDetector.h"

CollisionDetector::CollisionDetector(Polygon_2 robot1, Polygon_2 robot2, Obstacles* obs)
: approx_robot1(robot1)
, approx_robot2(robot2)
, m_obs(obs)
, m_translate_helper()
{
	for(int i = 0; i < approx_robot1.size(); ++i)
	{
		Vector_2 minus_p = CGAL::ORIGIN - approx_robot1.vertex(i);
		m_translate_helper.push_back(Point_2(minus_p.x(),minus_p.y()));
	}
	Polygon_2 minus_r1(m_translate_helper.begin(), m_translate_helper.end());
	m_translate_helper.resize(0);
	for(int i = 0; i < approx_robot2.size(); ++i)
	{
		Vector_2 minus_p = CGAL::ORIGIN - approx_robot2.vertex(i);
		m_translate_helper.push_back(Point_2(minus_p.x(),minus_p.y()));
	}
	Polygon_2 minus_r2(m_translate_helper.begin(), m_translate_helper.end());

	// Construction of the polygon set, used for collision detection
	if (!m_obs->empty()) 
	{
		for (Obstacles::iterator iter =
				m_obs->begin(); iter != m_obs->end(); iter++)
		{
			// For every obstacle calculate its Minkowski sum with the "robot"
			Polygon_with_holes_2  poly_wh1 = minkowski_sum_2(*iter, minus_r1);
			Polygon_with_holes_2  poly_wh2 = minkowski_sum_2(*iter, minus_r2);

			// Add the result to the polygon set
			m_r1_poly_set.join(poly_wh1);
			m_r2_poly_set.join(poly_wh2);
		}
	}
}

Polygon_2 CollisionDetector::translate_polygon_to(const Polygon_2& poly, const Point_2& new_ref_pt) const
{
	m_translate_helper.resize(0);
	const Point_2 &ref = *poly.left_vertex();
	std::pair<double,double> diff(
	//Vector_2 diff(
		CGAL::to_double(ref.x().exact()) - CGAL::to_double(new_ref_pt.x().exact()), 
		CGAL::to_double(ref.y().exact()) - CGAL::to_double(new_ref_pt.y().exact())
	);
  	for (Polygon_2::Vertex_const_iterator it = poly.vertices_begin(), it_end = poly.vertices_end(); it != it_end; ++it)
  	{
		m_translate_helper.push_back( Point_2(CGAL::to_double(it->x()) + diff.first, CGAL::to_double(it->y()) + diff.second ) );
		//translated.push_back( (*it) + diff );
  	}
  	Polygon_2 new_poly(m_translate_helper.begin(),m_translate_helper.end());
  	return new_poly;
}

bool CollisionDetector::do_moved_robots_interesct(
	const Polygon_2 &robot1, 
	const Polygon_2 &robot2, 
	const Point_2 &r1_new, 
	const Point_2& r2_new
) const
{
	Polygon_2 r1 = translate_polygon_to(robot1,r1_new);
	Polygon_set_2 ps_r1;
	if (!r1.is_counterclockwise_oriented()) r1.reverse_orientation();
	ps_r1.insert(r1);

	Polygon_2 r2 = translate_polygon_to(robot2,r2_new);
	Polygon_set_2 ps_r2;
	if (!r2.is_counterclockwise_oriented()) r2.reverse_orientation();
	ps_r2.insert(r2);

	return ps_r1.do_intersect(ps_r2);
}

bool CollisionDetector::valid_conf( const Point_d &pos ) const
{
	//Point
	bool   res   =   false;
	Point_2 robo1_p( pos.cartesian(0),pos.cartesian(1) );
	Point_2 robo2_p( pos.cartesian(2),pos.cartesian(3) );

	bool is_robo1_p_valid = one_robot_valid_conf(robo1_p, m_r1_poly_set);
	bool is_robo2_p_valid = one_robot_valid_conf(robo2_p, m_r2_poly_set);

	return ( is_robo1_p_valid && is_robo2_p_valid );
	{
		//check that robos do not overlap, translate robos to new location

		res = !do_moved_robots_interesct( approx_robot1, approx_robot2,
				     	  	  	  	  	robo1_p, robo2_p );
	}

	return res;
}