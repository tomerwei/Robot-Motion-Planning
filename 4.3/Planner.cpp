#include "Planner.h"
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/point_generators_2.h>
#include "Kd_tree_d.h"
#include <boost/make_shared.hpp>
#include <CGAL/Boolean_set_operations_2.h>
#include <cassert>

using namespace std;

Planner::Planner(Scene* scene, int time, bool measure, double alpha, vector<vector<Conf>>* path, double* quality) : m_scene(scene)
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

	CollisionDetector m_collision( robot_poly1, robot_poly2, &m_obstacles );
	Sampler           m_sampler( robot_poly1, robot_poly2, m_room, m_collision );

    // An example


      Prm roadmap( 300, 12, m_collision,
                         m_sampler, curr_start_conf, curr_end_conf);
      roadmap.generate_roadmap();
      //loop end

    // retrieve path from PRM
    vector<Point_d> path = roadmap.retrieve_path();


    // transform path to GUI and update the display in gui
    //transfrom_path(path);


	/*	example of a dummy path that moves the robots from the start positions
		to target, and back to start */
	//m_path.push_back();
	m_path.resize(path.size());
	for(int i(0), sz(path.size()); i < sz; ++i)
	{
		m_path[i].push_back(Point_2(path[i].cartesian(0),path[i].cartesian(1)));
		m_path[i].push_back(Point_2(path[i].cartesian(2),path[i].cartesian(3)));
	}

	//	run this method when you finish to produce the path
	//	IMPORTANT: the result should be put in m_path
	transform_path_for_gui();
}
