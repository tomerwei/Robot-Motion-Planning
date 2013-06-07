#ifndef PLANNER_H
#define PLANNER_H

#include <QPointF>
#include <QVector>
#include <QPolygonF>
#include "basic_typedef.h"
#include "Scene.h"
#include "Graph.h"
#include "Kd_tree_d.h"
#include <CGAL/Random.h>
#include <boost/shared_ptr.hpp>
#include "CollisionDetector.h"
#include "Sampler.h"
#include "Prm.h"


class Planner
{
public:
	
	////////////////////////
	// C'tors & D'tors
	////////////////////////

	Planner(Scene* scene, int time, bool measure, double alpha, vector<vector<Conf>>* path, double* quality);

	~Planner(void);

	////////////////////////
	// modifiers
	////////////////////////

	void run();

	////////////////////////
	// helpers
	////////////////////////

private:
	Point_2		transformToCGAL(QPointF qp)
	{
		// x and y coordinates of the point
		double x = qp.x();
		double y = qp.y();

		Point_2 cpoint(x,y);
		return cpoint;
	}

	QPointF		transformToQT(Conf cp)
	{
		return QPointF(CGAL::to_double(cp.x()) , CGAL::to_double(cp.y()) );
	}

	QVector<QPointF> transformToQT(ConfSet cs)
	{
		QVector<QPointF> result;

		foreach(Conf c, cs)
		{
			result.push_back(transformToQT(c));
		}
		return result;
	}

	QVector<QVector<QPointF> > transformToQT(list<vector<Conf> > path)
	{
		QVector<QVector<QPointF> > result;

		foreach(ConfSet cs, path)
		{
			result.push_back(transformToQT(cs));
		}
		return result;
	}

	void		transform_path_for_gui()
	{
		vector<vector<vector<QConf> > > result_vectors(m_path.size());

		for (int i = 0; i < m_path.size(); i++)
		{
			for (int r = 0; r < m_robot_num; r++)
			{
				vector<QConf> path_for_r;

				QPointF vec;
				double angle = 0;
				if (i != 0)
				{
					QPointF s = transformToQT(m_path[i-1][r]);
					QPointF t = transformToQT(m_path[i][r]);
					vec = t - s;
				}
				else
					vec = transformToQT(m_path[i][r]);

				QConf qconf(vec,angle);
				path_for_r.push_back(qconf);

				result_vectors[i].push_back(path_for_r);
			}
		}
	
		/*	If "result" is empty, the GUI assumes that a solution haven't been found */
		m_scene->setPath(result_vectors);	
	}

	void	extract_scenario_from_gui()
	{
		Point_2 ptl = transformToCGAL(m_scene->getRoomTopLeft());
		Point_2 pbr = transformToCGAL(m_scene->getRoomBottomRight());
		Point_2 pbl(ptl.x(), pbr.y());
		Point_2 ptr(pbr.x(), ptl.y());
		m_room = Room(ptl,pbr);

		QVector<QPolygonF> obstacles = m_scene->getObstacles();

		vector<QPolygonF> robots = m_scene->getRobotPolygons();
		foreach(QPolygonF qp, robots)
		{
			Polygon_2 cp;
			for (int j=0; j < qp.size(); j++)
			{
				cp.push_back(transformToCGAL(qp[j]));
			}

			m_robot_polygons.push_back(cp);
		}

		foreach(QPolygonF qp, obstacles)
		{
			Polygon_2 cp;
			for (int j=0; j < qp.size(); j++)
			{
				cp.push_back(transformToCGAL(qp[j]));
			}

			m_obstacles.push_back(cp);
		}

		Polygon_2 wall_left;
		wall_left.push_back(pbl);
		wall_left.push_back(ptl);
	
		Polygon_2 wall_top;
		wall_top.push_back(ptl);
		wall_top.push_back(ptr);

		Polygon_2 wall_right;
		wall_right.push_back(ptr);
		wall_right.push_back(pbr);

		Polygon_2 wall_bot;
		wall_bot.push_back(pbl);
		wall_bot.push_back(pbr);

		m_obstacles.push_back(wall_left);
		m_obstacles.push_back(wall_top);
		m_obstacles.push_back(wall_right);
		m_obstacles.push_back(wall_bot);
	
		QVector<QVector<QPointF> > startPositions = m_scene->getStartPositions();
		QVector<QVector<QPointF> > targetPositions = m_scene->getTargetPositions();

		m_robot_num = targetPositions.size();

		for (int g = 0; g < m_robot_num; g++)
		{
			m_start_confs.push_back(transformToCGAL(startPositions[g][0]));
			m_target_confs.push_back(transformToCGAL(targetPositions[g][0]));
		}
	}

	///////////////////////
	// Data members
	///////////////////////
private:
	enum opt_type{
		OPT_TYPE_DISTANCE,
		OPT_TYPE_COMBO
	};

	Scene*						m_scene;
	vector<Polygon_2>			m_robot_polygons;
	Obstacles					m_obstacles;	// polygonal obstacles + scenario boundaries
	vector<Conf>				m_start_confs, m_target_confs;
	Room						m_room;
	int							m_robot_num;
	vector<vector<Conf> >		m_path;			// path section - robot - configuration
	opt_type					m_what_to_optimize;
	double						m_alpha;
	int							m_seconds;
};


#endif
