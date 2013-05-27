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
	void populate_robot_bounds(const Polygon_2 &robot, robot_bounds& bounds)
	{
		const Point_2 &maxX1 = *robot.right_vertex();
		const Point_2 &maxY1 = *robot.top_vertex();
		const Point_2 &minX1 = *robot.left_vertex();
		const Point_2 &minY1 = *robot.bottom_vertex();

		CGAL::Lazy_exact_nt<CGAL::Gmpq> xmin = tl.x() + minX1.x();
		bounds.xMin = CGAL::to_double(xmin);
		CGAL::Lazy_exact_nt<CGAL::Gmpq> xmax = br.x() - maxX1.x();
		bounds.xMax = CGAL::to_double(xmax);
		CGAL::Lazy_exact_nt<CGAL::Gmpq> ymax = br.y() - maxY1.y();
		bounds.yMax = CGAL::to_double(ymax);
		CGAL::Lazy_exact_nt<CGAL::Gmpq> ymin = tl.y() + minY1.y();
		bounds.yMin = CGAL::to_double(ymin);
		bounds.xLen = bounds.xMax - bounds.xMin;
		bounds.yLen = bounds.yMax - bounds.yMin;

	}

public:
  Sampler(//double radius,
		   Polygon_2 robot1,
		   Polygon_2 robot2,
		   Room room, CollisionDetector *col, double seed = -1) :
    //m_radius(radius),
	m_robot1( robot1 ),
	m_robot2( robot2 ),
    m_col(col), m_room(room)
  {  
    tl = m_room.first;
    br = m_room.second;

    //TODO -

	populate_robot_bounds(m_robot1,r1);
	populate_robot_bounds(m_robot2,r2);

    if (seed == -1) seed = time(NULL);
    srand(seed);
  }

  ~Sampler(void) { }

  ////////////////////////
  // Queries
  ////////////////////////
  
  /* Returns a point that represents a valid (free) configuration of the robot.
   */
  Point_d generate_sample()
  {
    {
      while(true) {
        // sample a point in the configuration space
        Point_d p = generate_sample_no_obstacles();

        // validate sample using a collision detector
        if (m_col->valid_conf(p)) return p;
      }
    }
  }

private:
  /* Samples a point in the configuration space of the robot defined by the
   * bounding box of the scenario. This point can be valid or invalid.
   */
  Point_d generate_sample_no_obstacles()
  {
	  std::vector<double> coords;
	  /*double randx1 = */coords.push_back(r1.xMin + r1.xMax * ((double)rand())/((double)RAND_MAX));
	  /*double randy1 = */coords.push_back(r1.yMin + r1.yLen * ((double)rand())/((double)RAND_MAX));
	  /*double randx2 = */coords.push_back(r2.xMin + r2.xLen * ((double)rand())/((double)RAND_MAX));
	  /*double randy2 = */coords.push_back(r2.yMin + r2.yLen * ((double)rand())/((double)RAND_MAX));

    Point_d p(4,coords.begin(),coords.end());

    return p;
  }

  ////////////////////////
  // Data Members
  ////////////////////////
  //double m_radius;              // radius of robots

  Polygon_2 m_robot1;
  Polygon_2 m_robot2;
  Room m_room;                  // Represents the boundaries of the scenario
  CollisionDetector* m_col;     // Collision detector

  // room variables
  Point_2 tl;
  Point_2 br;

  robot_bounds r1;
  robot_bounds r2;
};
#endif
