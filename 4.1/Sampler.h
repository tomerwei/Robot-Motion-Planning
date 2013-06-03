#ifndef SAMPLER_H
#define SAMPLER_H

#include "basic_typedef.h"
#include "CollisionDetector.h"
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Gmpq.h>
#include <CGAL/number_utils.h>

class Sampler {
public:
  ////////////////////////
  // CTORS
  ////////////////////////
  /*  Parameters:
      radius  - radius of robot
      Room  - describes the boundary of the scenario
      CollisionDetector  - Collision detection black box
  */

private:
	struct robot_bounds{
	  double xMin, xMax, yMin,yMax;
	  double xLen,yLen;
	};
	void populate_robot_bounds(const Polygon_2 &robot, robot_bounds& bounds);

public:
  Sampler(//double radius,
		   Polygon_2 robot1,
		   Polygon_2 robot2,
		   Room room, const CollisionDetector &col, double seed = -1);

  ~Sampler(void) { }

  ////////////////////////
  // Queries
  ////////////////////////
  
  /* Returns a point that represents a valid (free) configuration of the robot.
   */
  Point_d generate_sample() const;

private:
  /* Samples a point in the configuration space of the robot defined by the
   * bounding box of the scenario. This point can be valid or invalid.
   */
  Point_d generate_sample_no_obstacles() const;

  ////////////////////////
  // Data Members
  ////////////////////////
  //double m_radius;              // radius of robots

  Polygon_2 m_robot1;
  Polygon_2 m_robot2;
  Room m_room;                  // Represents the boundaries of the scenario
  const CollisionDetector &m_col;     // Collision detector

  // room variables
  Point_2 tl;
  Point_2 br;

  robot_bounds r1;
  robot_bounds r2;
};
#endif
