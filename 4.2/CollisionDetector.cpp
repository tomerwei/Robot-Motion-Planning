#include "CollisionDetector.h"

CollisionDetector::CollisionDetector(Polygon_2 robot, Obstacles* obs)
: approx_robot(robot)
, m_obs(obs)
, m_translate_helper()
{
	for(int i = 0; i < approx_robot.size(); ++i)
	{
		Vector_2 minus_p = CGAL::ORIGIN - approx_robot.vertex(i);
		m_translate_helper.push_back(Point_2(minus_p.x(),minus_p.y()));
	}
	Polygon_2 minus_r1(m_translate_helper.begin(), m_translate_helper.end());
	
	// Construction of the polygon set, used for collision detection
	if (!m_obs->empty()) 
	{
		for (Obstacles::iterator iter =
				m_obs->begin(); iter != m_obs->end(); iter++)
		{
			// For every obstacle calculate its Minkowski sum with the "robot"
			Polygon_with_holes_2  poly_wh1 = minkowski_sum_2(*iter, minus_r1);

			// Add the result to the polygon set
			m_r1_poly_set.join(poly_wh1);
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

bool CollisionDetector::valid_conf( const Point_2 &pos ) const
{
	//Point
	bool is_robo1_p_valid = one_robot_valid_conf(pos, m_r1_poly_set);

	return is_robo1_p_valid;
}