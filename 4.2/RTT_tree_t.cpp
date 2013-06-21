#include "RTT_tree_t.h"

RRT_tree_t::RRT_tree_t(const std::vector<Conf>& tree_root, const SRPrm& r1_roadmap, const SRPrm& r2_roadmap, const CollisionDetector& collision_detector, const Sampler& sampler)
: m_root(tree_root)
, m_knn_container()
, m_tree()
, m_r1_roadmap(r1_roadmap)
, m_r2_roadmap(r2_roadmap)
, m_cd(collision_detector)
, m_sampler(sampler)
, m_cnv()
, m_knn_out()
{
}

Vector_2 RRT_tree_t::make_random_direction_vec()
{
	return Vector_2((double)rand()/RAND_MAX,(double)rand()/RAND_MAX);
}

Point_2 /*RRT_tree_t::*/get_nn_by_direction(
	const std::vector<Point_2>& pts,
	const Point_2& from,
	const Vector_2& dir
)
{
	double max_cos_angle = 0;
	Point_2 nearest;
	for(std::vector<Point_2>::const_iterator n_begin(pts.begin()), n_end(pts.end()); n_begin != n_end; ++n_begin)
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

		std::vector<Point_2> pts;
		m_r1_roadmap.get_neighbors(r1_pt, std::back_inserter(pts));

		Point_2 nn1_pt = get_nn_by_direction(pts,r1_pt,r1_direction);

		pts.resize(0);
		m_r2_roadmap.get_neighbors(r2_pt, std::back_inserter(pts));
	
		Point_2 nn2_pt = get_nn_by_direction(pts,r2_pt,r2_direction);

		if (!m_cd.do_moved_robots_interesct(nn1_pt,nn2_pt))
		{
			return to_pointd(nn1_pt,nn2_pt);
		}
	}
}

Point_d RRT_tree_t::to_pointd(const Point_2& r1, const Point_2& r2)
{
	m_cnv.resize(0);
	m_cnv.push_back(CGAL::to_double(r1.x()));
	m_cnv.push_back(CGAL::to_double(r1.y()));
	m_cnv.push_back(CGAL::to_double(r2.x()));
	m_cnv.push_back(CGAL::to_double(r2.y()));
	return Point_d(4,m_cnv.begin(),m_cnv.end());
}

void RRT_tree_t::expand(size_t samples)
{
	for(size_t i(0); i < samples; ++i)
	{
		Point_d q_rand = m_sampler.generate_sample_no_obstacles();
		Point_d q_near = virtual_graph_nearest_neighbor(q_rand);//m_knn_container.nearest_neighbor(q_rand);
		Point_d q_new = new_from_direction_oracle(q_near);

		if (!m_tree.is_in_graph(q_new))
		{
			m_tree.add_vertex(q_new);
			m_tree.add_edge(q_near,q_new);
		}
	}
}

Point_d RRT_tree_t::virtual_graph_nearest_neighbor(const Point_d &pt)
{
	const Point_2 r1 = m_r1_roadmap.get_nearest_to(Point_2(pt.cartesian(0), pt.cartesian(1))),
		r2 = m_r2_roadmap.get_nearest_to(Point_2(pt.cartesian(2),pt.cartesian(3)));
	return this->to_pointd(r1,r2);
}

const Point_d RRT_tree_t::get_nearest(const Point_d& nearest_to) const
{
	return m_knn_container.nearest_neighbor(nearest_to);
}