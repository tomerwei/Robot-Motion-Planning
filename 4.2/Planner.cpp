#include "Planner.h"
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
	size_t num_samples(100);


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

	CollisionDetector cdetector( robot_poly1, robot_poly2, &m_obstacles );
	Sampler sampler( robot_poly1, robot_poly2, m_room, cdetector );
	
	RRT_tree_t src_tree(m_start_confs, sampler);
	RRT_tree_t tgt_tree(m_target_confs, sampler);
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