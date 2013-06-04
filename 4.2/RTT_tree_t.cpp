#include "RTT_tree_t.h"

RRT_tree_t::RRT_tree_t(const std::vector<Conf>& tree_root, const Prm& r1_roadmap, const Prm& r2_roadmap)
: m_root(tree_root)
, m_knn_container()
, m_tree()
, m_r1_roadmap(r1_roadmap)
, m_r2_roadmap(r2_roadmap)
{
}

Vector_2 RRT_tree_t::make_random_direction_vec()
{
	return Vector_2((double)rand()/RAND_MAX,(double)rand()/RAND_MAX);
}

unsigned int /*RRT_tree_t::*/get_nn_by_direction(
	Graph<int, Less_than_int>::adjacency_iterator n_begin, 
	Graph<int, Less_than_int>::adjacency_iterator n_end,
	const Point_2& from,
	const Vector_2& dir
)
{
	double max_cos_angle = 0;
	unsigned int nearest;
	for(; n_begin != n_end; ++n_begin)
	{
		Vector_2 vec(from,*n_begin);
		double mag_dir = CGAL::to_double(dir * dir),
			mag_vec = CGAL::to_double(vec * vec);
		double cos_angle = CGAL::to_double(dir*vec) / sqrt(mag_dir * mag_vec);
		if (cos_angle > max_cos_angle)
		{
			max_cos_angle = cos_angle;
			nearest = *n_begin;
		}
	}

	return nearest;
}

Point_d RRT_tree_t::new_from_direction_oracle(const Point_d &pt)
{
	while (true) {
		//the full q is (r1_direction.x,r1_direction.y,r2_direction.x,r2_direction.y)
		Vector_2 r1_direction = make_random_direction_vec();
		Vector_2 r2_direction = make_random_direction_vec();

		Point_2 r1_pt(pt.cartesian(0),pt.cartesian(1));
		Point_2 r2_pt(pt.cartesian(2),pt.cartesian(3));

		std::pair<typename Graph<int, Less_than_int>::adjacency_iterator,
				  typename Graph<int, Less_than_int>::adjacency_iterator> r1_neighbor_range =
			m_r1_roadmap.get_neighbors(r1_pt);

		unsigned int nn1 = get_nn_by_direction(r1_neighbor_range.first,r1_neighbor_range.second,r1_pt,r1_direction);
		Point_2 nn1_pt = m_r1_roadmap.get_point_from_graph_id(nn1);

	
		std::pair<typename Graph<int, Less_than_int>::adjacency_iterator,
				  typename Graph<int, Less_than_int>::adjacency_iterator> r2_neighbor_range =
			m_r2_roadmap.get_neighbors(r2_pt);
	
		unsigned int nn2 = get_nn_by_direction(r2_neighbor_range.first,r2_neighbor_range.second,r2_pt,r2_direction);
		Point_2 nn2_pt = m_r2_roadmap.get_point_from_graph_id(nn2);

		std::vector<Kernel::RT> pt_vec;
		pt_vec.push_back(nn1_pt.x());
		pt_vec.push_back(nn1_pt.y());
		pt_vec.push_back(nn2_pt.x());
		pt_vec.push_back(nn2_pt.y());
		Point_d nn_pt(4, pt_vec.begin(),pt_vec.end());

		if (no_robot_collision(pt,nn_pt)) {
			return nn_pt;
		}
	}
}

void RRT_tree_t::expand(size_t samples)
{
	for(size_t i(0); i < samples; ++i)
	{
		Point_d q_rand = make_random_sample();
		Point_d q_near = virtual_graph_nearest_neighbor(q_rand);//m_knn_container.nearest_neighbor(q_rand);
		Point_d q_new = new_from_direction_oracle(q_near);

		if (!m_tree.is_in_graph(q_new))
		{
			m_tree.add_vertex(q_new);
			m_tree.add_edge(q_near,q_new);
		}
	}
}

const Point_d RRT_tree_t::get_nearest(const Point_d& nearest_to) const
{
	return m_knn_container.nearest_neighbor(nearest_to);
}