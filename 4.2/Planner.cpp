#include "Planner.h"
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/point_generators_2.h>
#include "Kd_tree_d.h"
#include <boost/make_shared.hpp>
#include <CGAL/Boolean_set_operations_2.h>

using namespace std;

Planner::Planner(Scene* scene) : m_scene(scene)
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
	/*	example of a dummy path that moves the robots from the start positions
		to target, and back to start */
	m_path.push_back(m_start_confs);
	m_path.push_back(m_target_confs);
	m_path.push_back(m_start_confs);

	//	run this method when you finish to produce the path
	//	IMPORTANT: the result should be put in m_path
	transform_path_for_gui();
}