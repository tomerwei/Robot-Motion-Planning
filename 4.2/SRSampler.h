#ifndef SRSAMPLER_H
#define SRSAMPLER_H

#include "basic_typedef.h"
#include "SRCollisionDetector.h"

class SRSampler {
public:
  ////////////////////////
  // CTORS
  ////////////////////////
  /*  Parameters:
      radius  - radius of robot
      Room  - describes the boundary of the scenario
      CollisionDetector  - Collision detection black box
  */

  SRSampler(const Polygon_2& robot, Room room, const SRCollisionDetector &col, double seed = -1) : 
    m_col(col), m_room(room), m_robot(robot)
{
	const Point_2 &maxX1 = *robot.right_vertex();
	const Point_2 &maxY1 = *robot.top_vertex();
	const Point_2 &minX1 = *robot.left_vertex();
	const Point_2 &minY1 = *robot.bottom_vertex();
    tl = m_room.first;
    br = m_room.second;
    CGAL::Lazy_exact_nt<CGAL::Gmpq> xmin = tl.x() + minX1.x();
	this->xmin = CGAL::to_double(xmin);
	CGAL::Lazy_exact_nt<CGAL::Gmpq> xmax = br.x() - maxX1.x();
	this->xmax = CGAL::to_double(xmax);
	CGAL::Lazy_exact_nt<CGAL::Gmpq> ymax = br.y() - maxY1.y();
	this->ymax = CGAL::to_double(ymax);
	CGAL::Lazy_exact_nt<CGAL::Gmpq> ymin = tl.y() + minY1.y();
	this->ymin = CGAL::to_double(ymin);
	this->lenx = this->xmax - this->xmin;
	this->leny = this->ymax - this->ymin;

    if (seed == -1) seed = time(NULL);
    srand(seed);
  }

  ~SRSampler(void) { }

  ////////////////////////
  // Queries
  ////////////////////////
  
  /* Returns a point that represents a valid (free) configuration of the robot.
   */
  Point_2 generate_sample() const
  {
    {
      while(true) {
        // sample a point in the configuration space
        Point_2 p = generate_sample_no_obstacles();

        // validate sample using a collision detector
        if (m_col.valid_conf(p)) return p;
      }
    }
  }

private:
  /* Samples a point in the configuration space of the robot defined by the
   * bounding box of the scenario. This point can be valid or invalid.
   */
  Point_2 generate_sample_no_obstacles() const
  {
    double randx = xmin + lenx * ((double)rand())/((double)RAND_MAX);
    double randy = ymin + leny * ((double)rand())/((double)RAND_MAX);

    Point_2 p(randx, randy);

    return p;
  }

  ////////////////////////
  // Data Members
  ////////////////////////    
  Room m_room;                  // Represents the boundaries of the scenario
  const SRCollisionDetector &m_col;     // Collision detector
  Polygon_2 m_robot;

  // room variables
  Point_2 tl;
  Point_2 br;
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double lenx;
  double leny;
};
#endif
