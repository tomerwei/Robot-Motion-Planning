#ifndef COLLLISION_DETECTOR_H
#define COLLLISION_DETECTOR_H

#include "basic_typedef.h"

class CollisionDetector {
public:  
  ////////////////////////
  // CTORS
  ////////////////////////
  /* Parameters:
   * radius  - radius of robot
   * Room  - describes the boundary of the scenario
   */
  CollisionDetector(Polygon_2 robot1, Polygon_2 robot2, Obstacles* obs, double eps);
  
  ~CollisionDetector() {};

  ////////////////////////
  // Queries
  ////////////////////////


  Polygon_2 translate_polygon_to(const Polygon_2& poly, const Point_2& new_ref_pt) const;
  Polygon_2 CollisionDetector::flip(const Polygon_2& robot);

  bool do_moved_robots_interesct( const Point_2 &robot1, const Point_2 &robot2 ) const;

  bool one_robot_valid_conf(const Point_2 &pos, const Polygon_set_2& poly_set ) const
  {
	  return (poly_set.oriented_side(pos) == CGAL::ON_NEGATIVE_SIDE);
  }

  /* Check if the given configuration is collision free (returns true for
   * free samples)
   */
  bool valid_conf( const Point_d &pos ) const;
  bool valid_conf( const Point_2& robo1_p, const Point_2& robo2_p) const;

  /* Validate the connection between two configurations by 
   * sampling along a line.
   * Input:  * Start and target configurations
   * Eps value that determines the size of the steps 
   * that are taken along the line
   */

  Obstacles* m_obs;             // Collection of polygonal obstacles
  Polygon_set_2 m_r1_poly_set,m_r2_poly_set;     // Polygon set of robots 1 and 2, for collision detection
  Polygon_2     approx_robot1;
  Polygon_2     approx_robot2;
  mutable std::vector<Point_2> m_translate_helper; //mutable so that the methods for do_valid can remain const
  const Polygon_2 m_minus_r1;
  const Polygon_2 m_minus_r2;
  Polygon_2 m_minus_r1_en,m_minus_r2_en; //enlarged versions of the robots
  Polygon_set_2 m_r1_min_r2, m_r2_min_r1;
  const double m_epsilon;
};

class LocalPlanner {
public: 
  explicit LocalPlanner(const CollisionDetector& cd);
  bool local_planner(const Point_d &start, const Point_d &target );
private:
  bool local_planner_one_robot(const Point_2& start, const Point_2& target, const Polygon_set_2& obstacles, double eps);
  bool local_planner_two_robot(const Point_2& start_r1, const Point_2& target_r1, const Point_2& start_r2, const Point_2& target_r2, double eps);

  ////////////////////////
  // Data Members
  ////////////////////////
	const CollisionDetector& m_cd;
};

#endif
