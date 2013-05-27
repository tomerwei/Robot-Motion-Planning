/*******************************************************************
*	File name: 		Scene.h
*	Description:	Implementation of the Scene class
*					The class wraps all the geometric data structures
*					needed for the motion planner for its processing
*					and for the GUI layer to display.
*					
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "Scene.h"
#include <QPropertyAnimation>
#include <iostream>
#include <fstream>
#include <QDataStream>
#include <qcolor.h>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <QVariantAnimation>
#include <QMetaType>

////////////////////////
// C'tors & D'tors
////////////////////////

Q_DECLARE_METATYPE(QPolygonF)

Scene::Scene(QObject * parent ) : QGraphicsScene(parent)
{
	connect(this, SIGNAL(selectionChanged()), this, SLOT(selectedUpdate()));
	m_current_robot = -1;
	add_grid();
}

Scene::~Scene(void)
{
}

	////////////////////////
	// Access methods
	////////////////////////

	////////////////////////
	// Modifiers
	////////////////////////
	
/* Adds the point qp to the obstacle currently drawn */
bool Scene::addPointToObstacle(QPointF qp)
{	
	return addPoint(qp,true);
}

/*	Sets the room	*/
void Scene::setRoom(QPointF topleft, QPointF bottomright)
{
	m_roomTL = topleft;
	m_roomBR = bottomright;
}

///////////////////////
// Helper methods
///////////////////////

/* Returns a rectangle with center point qp and height/width = radius*2 */
QRectF Scene::rectForPoint(QPointF qp, double radius){
	qreal  xCord = qp.rx() - radius;
	qreal  yCord = qp.ry() - radius;
	QRectF rectangle( xCord,  yCord,  radius*2, radius*2);
	return rectangle;
}

	////////////////////////
	// Other methods
	////////////////////////


void Scene::keyPressEvent (QKeyEvent* evt){
	
	bool isDeleteKey (evt->key() == Qt::Key_Delete);

}

QVariant interpolateQPolygonF(const QPolygonF &start, const QPolygonF &end, qreal progress)
{
    QPolygonF result(start);
	
	QPointF start_ref = Scene::reference_point(start);
	QPointF end_ref = Scene::reference_point(end);
	
	QPointF translation = (end_ref - start_ref) * progress;
	result.translate(translation);

	QLineF line_start(start[1],start[0]);
	QLineF line_end(end[1],end[0]);

	double angle_start = line_start.angle();
	double angle_end = line_end.angle();

	double angle_diff = (angle_start - angle_end) * progress;
	result = Scene::rotate_polygon(result, angle_diff);

    QVariant res;
    res.setValue(result);
    return res;
}

void Scene::animate()
	{
		const int groups_num = m_robot_polygon.size();
		vector<int>	robot_num;

		for (int g = 0; g < groups_num; g++)
		{
			robot_num.push_back(m_conf_rotation_start[g].size());
		}
		//Fill Vector
		qRegisterAnimationInterpolator<QPolygonF>(interpolateQPolygonF);
		vector<vector<GraphicsPolygon*> > graphics_polygon(groups_num);
		for (int g = 0; g < groups_num; g++)
		{
			for (int i = 0; i< robot_num[g]; i++)
				graphics_polygon[g].push_back(new GraphicsPolygon);
		}
		
		bool path_exists = true;
		
		// if path empty create "artificial path"
		if (m_path.empty())
		{
			vector<vector<vector<QConf> > >  path(2);
			for (int g = 0; g < groups_num; g++)
			{
				vector<QConf> path_section_group_start;
				vector<QConf> path_section_group_target;

				for (int r = 0; r < robot_num[g]; r++)
				{
					QPolygonF first_poly = m_disp_conf_start[g][r]->polygon();
					QPointF start_ref = reference_point(first_poly);

					QPointF map_start_ref = m_disp_conf_start[g][r]->mapToScene(start_ref);
					QConf start(map_start_ref, m_conf_rotation_start[g][r]);
					path_section_group_start.push_back(start);

					QPolygonF target_polygon = m_disp_conf_target[g][r]->polygon();
					QPointF target_ref = reference_point(target_polygon);
					QPointF map_target_ref = m_disp_conf_target[g][r]->mapToScene(target_ref);

					QConf target((map_target_ref - map_start_ref), 
							m_conf_rotation_start[g][r] - m_conf_rotation_target[g][r]);
					
					path_section_group_target.push_back(target);
				}
				path[0].push_back(path_section_group_start);
				path[1].push_back(path_section_group_target);
			}
			
			m_path = path;
			
			path_exists = false;
		} 

		// prepare (move robot to start configuration)
		vector<vector<QPolygonF> > polygon(groups_num);
		for (int g = 0; g < groups_num; g++)
		{
			for (int r = 0; r < robot_num[g]; r++)
			{
				polygon[g].push_back(m_robot_polygon[g]);

				QMatrix tranlate_to_start;
				QPointF translate_start_point = m_path[0][g][r].first - reference_point(polygon[g][r]);
				tranlate_to_start.translate(translate_start_point.x(), translate_start_point.y());
				polygon[g][r] = tranlate_to_start.map(polygon[g][r]);
				polygon[g][r] = rotate_polygon(polygon[g][r], m_path[0][g][r].second);
				graphics_polygon[g][r]->setPolygon(polygon[g][r]);
				graphics_polygon[g][r]->setBrush(QBrush(GROUP_COLORS[g],TARGET_STYLE));
				graphics_polygon[g][r]->setZValue(1);
				//graphics_polygon[g][r]->setPen(QPen(ROBOT_COLOR[r]));
			
				//Add To Scene the Graphic Polygon
				addItem(graphics_polygon[g][r]);
			}
		}
		
		QSequentialAnimationGroup* seq_animation = new QSequentialAnimationGroup;

		for (int i = 1; i < m_path.size(); i++)
		{
			QParallelAnimationGroup* par_animation_groups = new QParallelAnimationGroup;
			for (int g = 0; g < groups_num; g++)
			{
				QParallelAnimationGroup* par_animation_robots = new QParallelAnimationGroup;
				vector<QPropertyAnimation*> prop_animation(robot_num[g]);

				vector<int> robot_min;

				for (int r = 0; r < robot_num[g]; r++)
				{
					prop_animation[r] = new QPropertyAnimation(graphics_polygon[g][r],"Polygon");
					prop_animation[r]->setDirection(QAbstractAnimation::Forward);
				}

				int num_steps = 10;

				for (int r = 0; r < robot_num[g]; r++)
				{
					prop_animation[r]->setDuration(ANIMATION_STEP_LENGTH * num_steps);
					QVariant start_variant;
					start_variant.setValue(polygon[g][r]);
					prop_animation[r]->setStartValue(start_variant);

					QPointF translation_point = m_path[i][g][r].first;
					double  rotation_angle = m_path[i][g][r].second;

					QMatrix matrix_tranlate;
					matrix_tranlate.translate(translation_point.x(), translation_point.y());
					polygon[g][r] = matrix_tranlate.map(polygon[g][r]);

					polygon[g][r] = rotate_polygon(polygon[g][r], rotation_angle);

					QVariant end_variant;
					end_variant.setValue(polygon[g][r]);
					prop_animation[r]->setEndValue(end_variant);
					par_animation_robots->addAnimation(prop_animation[r]);
				}			

				par_animation_groups->addAnimation(par_animation_robots);

			}

			seq_animation->addAnimation(par_animation_groups);
		}
		
		QObject::connect(seq_animation,SIGNAL( finished() ),this,SLOT( animationComplete() ));
		const int dummy_int = groups_num;
		m_displayed_animation_objects = QVector<QVector<QGraphicsPolygonItem*> > (dummy_int);
		for (int g = 0; g < groups_num; g++)
		{
			for (int r = 0; r < robot_num[g]; r++)
			{
				m_displayed_animation_objects[g].push_back(graphics_polygon[g][r]);
			}
		}
		

		seq_animation -> start();
		if (!path_exists)
			m_path.clear();
	}
