#ifndef COLLLISION_DETECTOR_H
#define COLLLISION_DETECTOR_H

#include "basic_typedef.h"
#include <CGAL/squared_distance_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/intersections.h>
#include <CGAL/convex_hull_2.h>

class CollisionDetector {
public:  
  ////////////////////////
  // CTORS
  ////////////////////////
  /* Parameters:
   * radius  - radius of robot
   * Room  - describes the boundary of the scenario
   */
  CollisionDetector(Polygon_2 robot1, Polygon_2 robot2, Obstacles* obs);
  
  ~CollisionDetector() {};

  ////////////////////////
  // Queries
  ////////////////////////


  Polygon_2 translate_polygon_to(const Polygon_2& poly, const Point_2& new_ref_pt) const;

  bool do_moved_robots_interesct(const Polygon_2 &robot1, const Polygon_2 &robot2, const Point_2 &r1_new, const Point_2& r2_new) const;

  bool one_robot_valid_conf(const Point_2 &pos, const Polygon_set_2& poly_set) const
  {
	  return poly_set.oriented_side(pos) == CGAL::ON_NEGATIVE_SIDE;
  }

  /* Check if the given configuration is collision free (returns true for
   * free samples)
   */
  bool valid_conf( const Point_d &pos ) const;

  /* Validate the connection between two configurations by 
   * sampling along a line.
   * Input:  * Start and target configurations
   * Eps value that determines the size of the steps 
   * that are taken along the line
   */
  bool local_planner_one_robot(const Point_2& start, const Point_2& target, const Polygon_set_2& obstacles, double eps)
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
      if (!one_robot_valid_conf(currentPos,obstacles)) return false;
    }

    // GREAT SUCCESS!
    return true;
  }

  Polygon_2 approx_mink(const Segment_2& seg, const Polygon_2& poly)
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

  bool local_planner(Point_d start, Point_d target, double eps )
  {
	  
	bool res = true;

	Point_2 robo1_start( start.cartesian(0),start.cartesian(1) );
	Point_2 robo2_start( start.cartesian(2),start.cartesian(3) );

	Point_2 robo1_target( target.cartesian(0),target.cartesian(1) );
	Point_2 robo2_target( target.cartesian(2),target.cartesian(3) );

	if( !local_planner_one_robot(robo1_start,robo1_target,m_r1_poly_set,eps) ||
		!local_planner_one_robot(robo2_start,robo2_target,m_r2_poly_set,eps) )
	{
		return false;
	}

	Kernel::Segment_2 r1_p( robo1_start,robo1_target );
	Polygon_2 r1_path = approx_mink( r1_p,approx_robot1 );
    Polygon_set_2 ps_r1_p;
    if ( !r1_path.is_counterclockwise_oriented() )
    {
    	r1_path.reverse_orientation();
    }
    ps_r1_p.insert( r1_path );


	Kernel::Segment_2 r2_p( robo2_start,robo2_target );
	Polygon_2 r2_path = approx_mink( r2_p,approx_robot2 );
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

  ////////////////////////
  // Data Members
  ////////////////////////
  Obstacles* m_obs;             // Collection of polygonal obstacles
  Polygon_set_2 m_r1_poly_set,m_r2_poly_set;     // Polygon set, for collision detection
  Polygon_2     approx_robot1;
  Polygon_2     approx_robot2;
  mutable std::vector<Point_2> m_translate_helper; //mutable so that the methods for do_valid can remain const
};

#endif
