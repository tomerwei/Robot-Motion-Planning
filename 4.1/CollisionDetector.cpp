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
	m_translate_helper.resize(0);
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
	const Polygon_2 &robot2
) const
{
	/*
	Polygon_2      r2    =  translate_polygon_to( robot2,r2_new );
	Polygon_set_2  ps_r2;
	if ( !r2.is_counterclockwise_oriented() )
	{
		r2.reverse_orientation();
	}
	ps_r2.insert( r2 );

	Polygon_with_holes_2  poly_wh1 = minkowski_sum_2( r2, m_minus_r1 );
	Polygon_set_2 ps;
	ps.insert( poly_wh1 );

	return ps.oriented_side(r1_new) == CGAL::ON_BOUNDED_SIDE;
	*/

	CGAL::Polygon_set_2<Kernel> set;

    set.join( robot1 );
	set.intersection( robot2 );

	return !set.is_empty();
}

bool CollisionDetector::valid_conf( const Point_d &pos ) const
{
	//Point
	bool   res   =   false;
	Point_2 robo1_p( pos.cartesian(0),pos.cartesian(1) );
	Point_2 robo2_p( pos.cartesian(2),pos.cartesian(3) );

	Polygon_2  trans_r1     =  translate_polygon_to( approx_robot1,robo1_p );
	Polygon_2  trans_r2     =  translate_polygon_to( approx_robot2,robo2_p );

	m_translate_helper.resize(0);
	for(int i = 0; i < trans_r1.size(); ++i)
	{
		Vector_2 minus_p = CGAL::ORIGIN - trans_r1.vertex(i);
		m_translate_helper.push_back(Point_2(minus_p.x(),minus_p.y()));
	}
	Polygon_2 minus_trans_r2( m_translate_helper.begin(), m_translate_helper.end() );
	Polygon_with_holes_2  m_robo_r1 = minkowski_sum_2( trans_r1 , minus_trans_r2);

	m_translate_helper.resize(0);
	for(int i = 0; i < trans_r2.size(); ++i)
	{
		Vector_2 minus_p = CGAL::ORIGIN - trans_r2.vertex(i);
		m_translate_helper.push_back(Point_2(minus_p.x(),minus_p.y()));
	}
	Polygon_2 minus_trans_r1( m_translate_helper.begin(), m_translate_helper.end() );
	Polygon_with_holes_2  m_robo_r2 = minkowski_sum_2( trans_r2 , minus_trans_r1);

	bool      is_robo1_p_valid  =  one_robot_valid_conf(robo1_p, m_r1_poly_set, m_robo_r2 );
	bool      is_robo2_p_valid  =  one_robot_valid_conf(robo2_p, m_r2_poly_set, m_robo_r1 );

	if ( is_robo1_p_valid && is_robo2_p_valid )
	{
		//check that robos do not overlap, translate robos to new location

		res = !do_moved_robots_interesct( trans_r1, trans_r2 );
	}

	return res;
}

LocalPlanner::LocalPlanner(const CollisionDetector& cd)
	:m_cd(cd)
{}


bool LocalPlanner::local_planner_two_robot(const Point_2& start_r1,
		                                         const Point_2& target_r1,
		                                         const Point_2& start_r2,
		                                         const Point_2& target_r2,
		                                         double eps )
{
	//robot 1 calc
	double x1_r1 = CGAL::to_double(start_r1.x());
	double y1_r1 = CGAL::to_double(start_r1.y());
	double x2_r1 = CGAL::to_double(target_r1.x());
	double y2_r1 = CGAL::to_double(target_r1.y()) ;

	double distance_r1 = sqrt(pow(x1_r1 - x2_r1, 2) + pow(y1_r1 - y2_r1,2));
	double step_size = eps;

	// calculate how many steps are required for first robot
	int step_num_r1 = floor((distance_r1 - step_size) / step_size);
	double vx_r1 = x2_r1 - x1_r1;
	double vy_r1 = y2_r1 - y1_r1;


	//robot 2 calc
	double x1_r2 = CGAL::to_double(start_r2.x());
	double y1_r2 = CGAL::to_double(start_r2.y());
	double x2_r2 = CGAL::to_double(target_r2.x());
	double y2_r2 = CGAL::to_double(target_r2.y()) ;

	double distance_r2 = sqrt(pow(x1_r2 - x2_r2, 2) + pow(y1_r2 - y2_r2,2));

	// calculate how many steps are required for second robot
	int step_num_r2 = floor((distance_r2 - step_size) / step_size);
	double vx_r2 = x2_r2 - x1_r2;
	double vy_r2 = y2_r2 - y1_r2;

	//select the largest number of steps of both robot for loop iteration check
	double step_num =  step_num_r1 > step_num_r2 ? step_num_r1 : step_num_r2;

	double step_size_r1 = distance_r1/step_num;
	double step_size_r2 = distance_r2/step_num;

	std::vector<double> coords;

	for (int i = 1; i <= step_num; ++i)
	{
		// generate a configuration for every step
		double offset_r1 =  (i * step_size_r1) / (distance_r1 - step_size_r1);
		double currx_r1 = x1_r1 + vx_r1 * offset_r1;
		double curry_r1 = y1_r1 + vy_r1 * offset_r1;

		double offset_r2 =  (i * step_size_r2) / (distance_r2 - step_size_r2);
		double currx_r2 = x1_r2 + vx_r2 * offset_r2;
		double curry_r2 = y1_r2 + vy_r2 * offset_r2;

		coords.resize(0);
	    coords.push_back(currx_r1);
	    coords.push_back(curry_r1);
	    coords.push_back(currx_r2);
	    coords.push_back(curry_r2);
		Point_d p( 4,coords.begin(),coords.end() );

		if( !m_cd.valid_conf( p ) ) return false;

		// If an intermidiate configuration is invalid, return false
		//if (!m_cd.one_robot_valid_conf(currentPos,obstacles)) return false;
	}

	// GREAT SUCCESS!
	return true;
}


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

	for (int i = 1; i <= step_num; ++i)
	{
		// generate a configuration for every step
		double offset =  (i * step_size) / (distance - step_size);
		double currx = x1 + vx * offset;
		double curry = y1 + vy * offset;

		Point_2 currentPos(currx, curry);



		// If an intermidiate configuration is invalid, return false
		//if (!m_cd.one_robot_valid_conf(currentPos,obstacles)) return false;
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

	if( !local_planner_two_robot( robo1_start,robo1_target,
			                      robo2_start,robo2_target,
			                      eps ) )
	{
		return false;
    }

	cout << "Success for: " << robo1_start << "\t" << robo1_target << "\n";


	/*
	if( !local_planner_one_robot(robo1_start,robo1_target,m_cd.m_r1_poly_set,eps) ||
		!local_planner_one_robot(robo2_start,robo2_target,m_cd.m_r2_poly_set,eps) )
	{
		return false;
	}
	*/

	/*
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
	*/
	return true;
  }
