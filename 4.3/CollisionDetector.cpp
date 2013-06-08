#include "CollisionDetector.h"
#include <CGAL/squared_distance_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/intersections.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Small_side_angle_bisector_decomposition_2.h>

CollisionDetector::CollisionDetector(Polygon_2 robot1, Polygon_2 robot2, Obstacles* obs, double eps)
: approx_robot1(robot1)
, approx_robot2(robot2)
, m_obs(obs)
, m_translate_helper()
, m_minus_r1()
, m_epsilon(eps)
{
	int nOfEdges = 8;
	double radius = eps/2;
	double dAlpha = (360.0/nOfEdges)*CGAL_PI/180;
	Polygon_2 expander;
	for (int i = nOfEdges; i>0; i--) {
		double alpha = (i + 0.5)*dAlpha;
		double x = (radius/cos(dAlpha/2)) * cos(alpha) * 1.05;
		double y = (radius/cos(dAlpha/2)) * sin(alpha) * 1.05;
		expander.push_back(Point_2(x, y));
	}

	CGAL::Small_side_angle_bisector_decomposition_2<Kernel>  ssab_decomp;
	Polygon_with_holes_2  pwh = minkowski_sum_2 (robot1, expander, ssab_decomp);
	approx_robot1 = pwh.outer_boundary();
	pwh = minkowski_sum_2 (robot2, expander, ssab_decomp);
	approx_robot2 = pwh.outer_boundary();

	for(int i = 0; i < approx_robot1.size(); ++i)
	{
		Vector_2 minus_p = CGAL::ORIGIN - approx_robot1.vertex(i);
		m_translate_helper.push_back(Point_2(minus_p.x(),minus_p.y()));
	}
	m_minus_r1 = Polygon_2(m_translate_helper.begin(), m_translate_helper.end());
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
			Polygon_with_holes_2  poly_wh1 = minkowski_sum_2(*iter, m_minus_r1);
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
	Polygon_2 r2 = translate_polygon_to(robot2,r2_new);
	Polygon_set_2 ps_r2;
	if (!r2.is_counterclockwise_oriented()) r2.reverse_orientation();
	ps_r2.insert(r2);

	Polygon_with_holes_2  poly_wh1 = minkowski_sum_2(r2, m_minus_r1);
	Polygon_set_2 ps; ps.insert(poly_wh1);
	return ps.oriented_side(r1_new) == CGAL::ON_BOUNDED_SIDE;
}

bool CollisionDetector::valid_conf( const Point_d &pos ) const
{
	//Point
	
	Point_2 robo1_p( pos.cartesian(0),pos.cartesian(1) );
	Point_2 robo2_p( pos.cartesian(2),pos.cartesian(3) );
	return valid_conf(robo1_p,robo2_p);
}

bool CollisionDetector::valid_conf( const Point_2 &robo1_p, const Point_2 &robo2_p ) const
{
	bool   res   =   false;
	bool is_robo1_p_valid = one_robot_valid_conf(robo1_p, m_r1_poly_set);
	bool is_robo2_p_valid = one_robot_valid_conf(robo2_p, m_r2_poly_set);

	if ( is_robo1_p_valid && is_robo2_p_valid );
	{
		//check that robos do not overlap, translate robos to new location

		res = !do_moved_robots_interesct( approx_robot1, approx_robot2,
				     	  	  	  	  	robo1_p, robo2_p );
	}

	return res;
}

LocalPlanner::LocalPlanner(const CollisionDetector& cd)
	:m_cd(cd)
{}

bool LocalPlanner::local_planner_one_robot(const Point_2& start, const Point_2& target, const Polygon_set_2& obstacles, double eps)
{
	double x1 = CGAL::to_double(start.x());
	double y1 = CGAL::to_double(start.y());
	double x2 = CGAL::to_double(target.x());
	double y2 = CGAL::to_double(target.y()) ;

	double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2,2));

	double step_size = eps;

	// calculate how many steps are required
	int step_num = floor((distance - step_size) / step_size);
	double vx = x2 - x1;
	double vy = y2 - y1;

	for (int i = 1; i <= step_num; ++i) {
		// generate a configuration for every step
		double offset =  (i * step_size) / (distance - step_size);
		double currx = x1 + vx * offset;
		double curry = y1 + vy * offset;

		Point_2 currentPos(currx, curry);

		// If an intermidiate configuration is invalid, return false
		if (!m_cd.one_robot_valid_conf(currentPos,obstacles)) return false;
	}

	// GREAT SUCCESS!
	return true;
}

Polygon_2 LocalPlanner::approx_mink(const Segment_2& seg, const Polygon_2& poly)
{
	//decompose?
	std::vector<Point_2> pts;
	Vector_2 seg_start(seg.source().x(),seg.source().y());
	Vector_2 seg_end(seg.target().x(),seg.target().y());
	for(Polygon_2::Vertex_const_iterator it = poly.vertices_begin(); it != poly.vertices_end(); ++it)
	{
		pts.push_back((*it) + seg_start);
		pts.push_back((*it) + seg_end);
	}
	std::vector<Point_2> ret;
	CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(ret));
	return Polygon_2(ret.begin(), ret.end());
}

  bool LocalPlanner::local_planner(Point_d start, Point_d target )
  {  
	  double step_size = m_cd.m_epsilon;

	  double last_clearance = 0;

	  //check which robot has max distnace from s to t
	  Point_2 r1_s(start.cartesian(0), start.cartesian(1)),
		  r2_s(start.cartesian(2),start.cartesian(3)),
		  r1_t(target.cartesian(0),target.cartesian(1)),
		  r2_t(target.cartesian(2),target.cartesian(2));

	  double max_dist = std::max(
		  CGAL::to_double(CGAL::squared_distance(r1_s,r1_t)),
		  CGAL::to_double(CGAL::squared_distance(r2_s,r2_t))
		);

  	  // calculate how many steps are required
	  int step_num = floor((max_dist - step_size) / step_size);

  	  // generate a configuration for every step
	  Vector_2 vec1 = Vector_2(r1_s,r1_t)/step_num;
	  Vector_2 vec2 = Vector_2(r2_s,r2_t)/step_num;
	  Point_2 s1 = r1_s,
		  s2 = r2_s;
	  for (int j = 1; j <= step_num; ++j) 
	  {
		  Point_2 t1 = s1 + vec1;
		  Point_2 t2 = s2 + vec2;
		  
		  // If an intermidiate configuration is invalid, return false
		  if (!m_cd.valid_conf(t1,t2))
		  {
			return false;
			}

		  s1 = t1;
		  s2 = t2;
	  }


    // GREAT SUCCESS!
    return true;
  }