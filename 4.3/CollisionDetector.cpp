#include "CollisionDetector.h"
#include <CGAL/squared_distance_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/intersections.h>
#include <CGAL/convex_hull_2.h>

CollisionDetector::CollisionDetector(Polygon_2 robot1, Polygon_2 robot2, Obstacles* obs)
: approx_robot1(robot1)
, approx_robot2(robot2)
, m_obs(obs)
, m_translate_helper()
, m_minus_r1()
{
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
	bool   res   =   false;
	Point_2 robo1_p( pos.cartesian(0),pos.cartesian(1) );
	Point_2 robo2_p( pos.cartesian(2),pos.cartesian(3) );

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

  bool LocalPlanner::local_planner(Point_d start, Point_d target, double eps )
  {
	  
	bool res = true;

	Point_2 robo1_start( start.cartesian(0),start.cartesian(1) );
	Point_2 robo2_start( start.cartesian(2),start.cartesian(3) );

	Point_2 robo1_target( target.cartesian(0),target.cartesian(1) );
	Point_2 robo2_target( target.cartesian(2),target.cartesian(3) );

	if( !local_planner_one_robot(robo1_start,robo1_target,m_cd.m_r1_poly_set,eps) ||
		!local_planner_one_robot(robo2_start,robo2_target,m_cd.m_r2_poly_set,eps) )
	{
		return false;
	}

	Kernel::Segment_2 r1_p( robo1_start,robo1_target );
	Polygon_2 r1_path = approx_mink( r1_p,m_cd.approx_robot1 );
    Polygon_set_2 ps_r1_p;
    if ( !r1_path.is_counterclockwise_oriented() )
    {
    	r1_path.reverse_orientation();
    }
    ps_r1_p.insert( r1_path );


	Kernel::Segment_2 r2_p( robo2_start,robo2_target );
	Polygon_2 r2_path = approx_mink( r2_p,m_cd.approx_robot2 );
	Polygon_set_2 ps_r2_p;
	if( !r2_path.is_counterclockwise_oriented() )
	{
		r2_path.reverse_orientation();
	}
	ps_r2_p.insert( r2_path );

	return ps_r1_p.do_intersect( ps_r2_p );
	
	//return !CGAL::do_intersect(r1_path,r2_path);

	/*double robo1_xStart = start.cartesian(0);
	double robo1_yStart = start.cartesian(1);
	double robo1_xEnd   = target.cartesian(0);
	double robo1_yEnd   = target.cartesian(1);

	double distance1 = sqrt(pow( robo1_xStart - robo1_xEnd, 2) +
						pow( robo1_yStart - robo1_yEnd,2));


	double robo2_xStart = start.cartesian(2);
	double robo2_yStart = start.cartesian(3);
	double robo2_xEnd   = target.cartesian(2);
	double robo2_yEnd   = target.cartesian(3);

	double distance2 = sqrt(pow( robo2_xStart - robo2_xEnd, 2) +
						pow( robo2_yStart - robo2_yEnd,2));

	double step_size = eps;

	int robo1_step_num = floor((distance1 - step_size) / step_size);
	int robo2_step_num = floor((distance1 - step_size) / step_size);


    for (int i = 1; i <= robo1_step_num; ++i)
    {

      double robo1_vx = robo1_xEnd - robo1_xStart;
      double robo1_vy = robo1_yEnd - robo1_yStart;

      // generate a configuration for every step
      double robo1_offset =  (i * step_size) / (distance1 - step_size);
      double robo1_currx = robo1_xStart + robo1_vx * robo1_offset;
      double robo1_curry = robo1_yStart + robo1_vy * robo1_offset;

      //Point_d currentPos(currx, curry);
      for( int j = 1; j <= robo2_step_num; ++j)
      {
          double robo2_vx = robo2_xEnd - robo2_xStart;
          double robo2_vy = robo2_yEnd - robo2_yStart;

          // generate a configuration for every step
          double robo2_offset =  (j * step_size) / (distance2 - step_size);
          double robo2_currx = robo2_xStart + robo2_vx * robo2_offset;
          double robo2_curry = robo2_yStart + robo2_vy * robo2_offset;

		  std::vector<double> curr;
		  curr.push_back(robo1_currx);
		  curr.push_back(robo1_curry);
		  curr.push_back(robo2_currx);
		  curr.push_back(robo2_curry);
		  Point_d currentPos(4, curr.begin(),curr.end()  );

          if (!valid_conf( currentPos )) return false;
      }

      // If an intermidiate configuration is invalid, return false

    }
	return true;
	*/
	 /*
    
    */
  }