#ifndef SRCOLLLISION_DETECTOR_H
#define SRCOLLLISION_DETECTOR_H

#include "basic_typedef.h"


class SRCollisionDetector {
public:  
  ////////////////////////
  // CTORS
  ////////////////////////
  /* Parameters:
   * radius  - radius of robot
   * Room  - describes the boundary of the scenario
   */
  SRCollisionDetector(Polygon_2 robot, Obstacles* obs);
  
  ~SRCollisionDetector() {};

  ////////////////////////
  // Queries
  ////////////////////////
  
  /* Check if the given configuration is collision free (returns true for
   * free samples)
   */
  bool valid_conf(const Point_2 &pos) const
  {
    return (m_poly_set.oriented_side(pos) == CGAL::ON_NEGATIVE_SIDE);
  }

  /* Validate the connection between two configurations by 
   * sampling along a line.
   * Input:  * Start and target configurations
   * Eps value that determines the size of the steps 
   * that are taken along the line
   */
  bool local_planner(const Point_2 &start, const Point_2 &target, double eps) const;

  ////////////////////////
  // Data Members
  ////////////////////////
  Polygon_2 m_robot;              // radius of robot
  Obstacles* m_obs;             // Collection of polygonal obstacles
  Polygon_set_2 m_poly_set;     // Polygon set, for collision detection
};

#endif
