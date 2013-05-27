#include "Planner.h"
#include "RTT_tree_t.h"
#include "dRRT_tree_connector_t.h"
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/point_generators_2.h>
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
	RRT_tree_t src_tree(m_start_confs);
	RRT_tree_t tgt_tree(m_target_confs);
	src_tree.expand();
	src_tree.expand();
	dRRT_tree_connector_t connector(src_tree,tgt_tree);
	if (connector.is_connected())
	{
		m_path = get_path(src_tree, connector.lhs_conn_pt());
		m_path.push_back(get_path(connector));
		m_path.push_back(get_path(dst_tree, connector.rhs_conn_pt());
	}
	//	run this method when you finish to produce the path
	//	IMPORTANT: the result should be put in m_path
	transform_path_for_gui();
}