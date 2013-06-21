#include "SRCollisionDetector.h"
#include <CGAL/Small_side_angle_bisector_decomposition_2.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/squared_distance_2.h>

SRCollisionDetector::SRCollisionDetector(Polygon_2 robot, Obstacles* obs)
: m_robot(robot)
, m_obs(obs) 
, m_poly_set()
{
	std::vector<Point_2> minus_pts;
	for(size_t i = 0; i < robot.size(); ++i)
	{
		minus_pts.push_back(Point_2(-robot.vertex(i).x(), -robot.vertex(i).y()));
	}

	Polygon_2 minus_robot(minus_pts.begin(), minus_pts.end());
	CGAL::Small_side_angle_bisector_decomposition_2<Kernel>  ssab_decomp;
    // Construction of the polygon set, used for collision detection
    if (!m_obs->empty()) 
	{
      for (Obstacles::iterator iter =
             m_obs->begin(); iter != m_obs->end(); iter++)
      {
        // For every obstacle calculate its Minkowski sum with the "robot"
        Polygon_with_holes_2  poly_wh = minkowski_sum_2(*iter, minus_robot,ssab_decomp);

        // Add the result to the polygon set
        m_poly_set.join(poly_wh);
      }
    }
}


bool SRCollisionDetector::local_planner(const Point_2 &start, const Point_2 &target, double eps) const
{
    /*double x1 = start.x();
    double y1 = start.y() ;
    double x2 = target.x() ;
    double y2 = target.y() ;*/

	double distance = sqrt(CGAL::to_double(CGAL::squared_distance(start,target)));

    double step_size = eps;

    // calculate how many steps are required
    int step_num = floor((distance - step_size) / step_size);
	Vector_2 step_vec(target.x() - start.x(),target.y() - start.y());

    for (int i = 1; i <= step_num; ++i) {
      // generate a configuration for every step
      double offset =  (i * step_size) / (distance - step_size);

      Point_2 currentPos = start + (step_vec * offset);

      // If an intermidiate configuration is invalid, return false
      if (!valid_conf(currentPos)) return false;
    }

    // GREAT SUCCESS!
    return true;
  }