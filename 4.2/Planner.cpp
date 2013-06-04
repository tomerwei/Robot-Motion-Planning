#include "Planner.h"
#include "Prm.h"
#include "RTT_tree_t.h"
#include "dRRT_tree_connector_t.h"
#include "Sampler.h"
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
	size_t num_samples(300);


	assert(m_start_confs.size() == 2);
	Polygon_2 robot_poly1(m_robot_polygons[0]); //via loop on all m_robot_polygons
	Polygon_2 robot_poly2(m_robot_polygons[1]); //via loop on all m_robot_polygons

	CollisionDetector cdetector1( robot_poly1, &m_obstacles );
	Sampler sampler1( robot_poly1, m_room, cdetector1 );

	Prm roadmap1( num_samples, 12, cdetector1, sampler1, m_start_confs[0], m_target_confs[0]);

	CollisionDetector cdetector2( robot_poly2, &m_obstacles );
	Sampler sampler2( robot_poly2, m_room, cdetector2 );

	Prm roadmap2( num_samples, 12, cdetector1, sampler1, m_start_confs[1], m_target_confs[1]);

	//todo: check both roadmaps have a path?

	RRT_tree_t src_tree(m_start_confs, roadmap1,roadmap2);
	RRT_tree_t tgt_tree(m_target_confs, roadmap1,roadmap2);

	src_tree.expand(num_samples);
	src_tree.expand(num_samples);
	dRRT_tree_connector_t connector(src_tree,tgt_tree, sampler);
	if (connector.is_connected())
	{
		m_path = get_path(src_tree, connector.lhs_conn_pt());
		m_path.push_back(get_path(connector));
		m_path.push_back(get_path(tgt_tree, connector.rhs_conn_pt()));
	}
	//	run this method when you finish to produce the path
	//	IMPORTANT: the result should be put in m_path
	transform_path_for_gui();
}