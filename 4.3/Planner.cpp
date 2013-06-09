#include "Planner.h"
#include "HGraph.h"
#include "Clearance_detector_ie.h"
#include <CGAL/point_generators_2.h>
#include "Kd_tree_d.h"
#include <boost/make_shared.hpp>
#include <CGAL/Boolean_set_operations_2.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/time.hpp>
#include <cassert>
#include <limits>


using namespace std;

namespace {
	struct path_length_dmetric : public HGraph::distance_metric {
		virtual double do_measure(const Point_d& lhs, const Point_d& rhs) const
		{
			const Point_2 r1_lhs(lhs.cartesian(0),lhs.cartesian(1)),
				r2_lhs(lhs.cartesian(2),lhs.cartesian(3)),
				r1_rhs(rhs.cartesian(0),rhs.cartesian(1)),
				r2_rhs(rhs.cartesian(2),rhs.cartesian(3));

			double r1_sq = CGAL::to_double(CGAL::squared_distance(r1_lhs,r1_rhs)),
				r2_sq = CGAL::to_double(CGAL::squared_distance(r2_lhs,r2_rhs));
			return sqrt(r1_sq) + sqrt(r2_sq);
		}
	};

	struct combo_dmetric : public HGraph::distance_metric {
		combo_dmetric(double alpha, double epsilon, const Clearance_detector_ie& cd_r1, const Clearance_detector_ie& cd_r2)
			:m_alpha(alpha), m_epsilon(epsilon), m_cd_r1(cd_r1), m_cd_r2(cd_r2) {}

		double clearance_along_path(const Point_2& lhs, const Point_2& rhs, const Clearance_detector_ie& cd) const
		{
			const double lhs_x = CGAL::to_double(lhs.x()),
				lhs_y = CGAL::to_double(lhs.y()),
				rhs_x = CGAL::to_double(rhs.x()),
				rhs_y = CGAL::to_double(rhs.y());

			Point_2_ie pt(lhs_x,lhs_y);
			double ret = std::min(numeric_limits<double>().max(),cd.clearance(pt));
			double distance = CGAL::to_double(CGAL::squared_distance(lhs,rhs));
			Vector_2_ie vec = Vector_2_ie(pt,Point_2_ie(rhs_x,rhs_y)) * m_epsilon / distance;
			int num_steps = floor((distance - m_epsilon) / m_epsilon);
			for(int i = 0; i < num_steps; ++i)
			{
				pt = pt + vec;
				ret = std::min(ret,cd.clearance(pt));
			}
			
			ret = std::min(ret,cd.clearance(Point_2_ie(rhs_x,rhs_y)));

			return ret;
		}

		virtual double do_measure( const Point_d& lhs, const Point_d& rhs) const
		{
			const Point_2 r1_lhs(lhs.cartesian(0),lhs.cartesian(1)),
				r2_lhs(lhs.cartesian(2),lhs.cartesian(3)),
				r1_rhs(rhs.cartesian(0),rhs.cartesian(1)),
				r2_rhs(rhs.cartesian(2),rhs.cartesian(3));

			double r1_sq = CGAL::to_double(CGAL::squared_distance(r1_lhs,r1_rhs)),
				r2_sq = CGAL::to_double(CGAL::squared_distance(r2_lhs,r2_rhs));

			double r1_l = sqrt(r1_sq),
				r2_l = sqrt(r2_sq);
			
			double r1_c = clearance_along_path(r1_lhs,r1_rhs,m_cd_r1),
				r2_c = clearance_along_path(r1_lhs,r1_rhs,m_cd_r2);

			return (r1_l / pow(r1_c,m_alpha)) + (r2_l / pow(r2_c,m_alpha));
		}

	private:
		const double m_alpha, m_epsilon;
		const Clearance_detector_ie& m_cd_r1, &m_cd_r2;
	};
}


Planner::Planner(Scene* scene, int time, bool measure, double alpha, vector<vector<Conf> >* path, double* quality) 
: m_scene(scene)
,m_what_to_optimize (measure ? OPT_TYPE_COMBO : OPT_TYPE_DISTANCE)
,m_alpha (alpha)
,m_seconds (time)
,m_path_out(path)
,m_quality_out(quality)
{
	/*	this method extracts information regarding the scenario from the 
		gui and initializes the fields 
			m_robot_polygons
			m_obstacles
			m_start_confs, m_target_confs
			m_room
			m_robot_num
	*/
	extract_scenario_from_gui();
}

Planner::~Planner()
{}

/*	This function is invoked by the GUI and is assumed to update the resulting */
void Planner::run()
{
	const double  EPS   =  0.5;
    boost::posix_time::ptime starts = boost::posix_time::microsec_clock::local_time();

	//loop start
	assert(m_start_confs.size() == 2);
	Polygon_2 robot_poly1(m_robot_polygons[0]); //via loop on all m_robot_polygons
	Polygon_2 robot_poly2(m_robot_polygons[1]); //via loop on all m_robot_polygons

	std::vector<double> start;
	start.push_back(CGAL::to_double(m_start_confs[0].x()));
	start.push_back(CGAL::to_double(m_start_confs[0].y()));
	start.push_back(CGAL::to_double(m_start_confs[1].x()));
	start.push_back(CGAL::to_double(m_start_confs[1].y()));
	Point_d      curr_start_conf(4,start.begin(),start.end());
	std::vector<double> end;
	end.push_back(CGAL::to_double(m_target_confs[0].x()));
	end.push_back(CGAL::to_double(m_target_confs[0].y()));
	end.push_back(CGAL::to_double(m_target_confs[1].x()));
	end.push_back(CGAL::to_double(m_target_confs[1].y()));
	Point_d      curr_end_conf(4,end.begin(),end.end());

	CollisionDetector m_collision( robot_poly1, robot_poly2, &m_obstacles,  EPS);
	Sampler           m_sampler( robot_poly1, robot_poly2, m_room, m_collision );
	std::auto_ptr<HGraph::distance_metric> dm;
	std::auto_ptr<Clearance_detector_ie> r1_cd, r2_cd;
	if (m_what_to_optimize == OPT_TYPE_DISTANCE)
	{
		dm.reset( new path_length_dmetric() );
	}
	else //OPT_TYPE_COMBO
	{
		r1_cd.reset(new Clearance_detector_ie(&m_obstacles,robot_poly1,EPS));
		r2_cd.reset(new Clearance_detector_ie(&m_obstacles,robot_poly2,EPS));
		dm.reset( new combo_dmetric(m_alpha, EPS, *r1_cd, *r2_cd) );
	}
	LocalPlanner lp(m_collision);
	HGraph hgraph( curr_start_conf,curr_end_conf, *dm,lp );

    // An example
	int msec_passed = 0;
	do{ // timer loop

      Prm roadmap( 250, 12, m_collision,
                         m_sampler, curr_start_conf, curr_end_conf);
      roadmap.generate_roadmap();

	  std::cout << "path length: " << roadmap.retrieve_path().size() << std::endl;

	  if (!roadmap.retrieve_path().empty())
	  {
      
		// retrieve path from PRM
		hgraph.push_back(roadmap.retrieve_path());

		const std::list<Point_d> &path(hgraph.get_path());
		m_path.resize(path.size());
		std::list<Point_d>::const_iterator it(path.begin()), it_end(path.end());
		for(int i = 0; it != it_end; ++it, ++i)
		{
			m_path[i].resize(0);
			m_path[i].push_back(Point_2(it->cartesian(0),it->cartesian(1)));
			m_path[i].push_back(Point_2(it->cartesian(2),it->cartesian(3)));
		}
		
		//	run this method when you finish to produce the path
		//	IMPORTANT: the result should be put in m_path
		transform_path_for_gui();
		m_path_out->assign(m_path.begin(), m_path.end());
		*m_quality_out = hgraph.get_distance();
	  }
	  boost::posix_time::ptime ends = boost::posix_time::microsec_clock::local_time();
	  boost::posix_time::time_duration msdiff =  ends - starts;
	  msec_passed = msdiff.total_milliseconds();
	} //end timer loop
	while (msec_passed < m_seconds*1000);

	std::cout << "time's up" << std::endl;
	if (!m_path.empty()) {
		assert(m_path[0][0] == m_start_confs[0]);
		assert(m_path[0][1] == m_start_confs[1]);
		assert(m_path[m_path.size() - 1][0] == m_target_confs[0]);
		assert(m_path[m_path.size() - 1][1] == m_target_confs[1]);
	}
}
