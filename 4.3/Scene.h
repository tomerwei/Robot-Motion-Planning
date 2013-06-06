/*******************************************************************
*	File name: 		Scene.h
*	Description:	Declaration of the Scene class
*					The class wraps all the geometric data structures
*					needed for the motion planner for its processing
*					and for the GUI layer to display.
*					
*	Authors:		Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#ifndef MOTION_PLAN_SCENE_H
#define MOTION_PLAN_SCENE_H

#pragma once

#include "basic_typedef.h"
#include <math.h>
#include <vector>
#include <QPolygonF>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QKeyEvent>
#include <time.h>
#include "GraphicsPolygon.h"
#include <QPropertyAnimation>

#include <QPen>
#include "colors.h"
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <iostream>
#include <fstream>
#include "get_bounding_circle_center.h"

const double POINT_RADIUS = 1.5;
const double PRECISION = 25;
const int ANIMATION_STAGE_DURATION = 500;
const int ANIMATION_STEP_LENGTH = 50;

class Scene : public QGraphicsScene
{
	Q_OBJECT

public:
	
	////////////////////////
	// C'tors & D'tors
	////////////////////////

	Scene(QObject * parent = 0);

	~Scene(void);


	////////////////////////
	// Access methods
	////////////////////////

	////////////////////////
	// Modifiers
	////////////////////////
	/*  Adds the point qp to the obstacle currently drawn */
	bool addPointToObstacle(QPointF qp);

	bool addPoint(QPointF qp, bool obs)
	{
		QPointF qplast,qpfirst;
		bool noPoints = m_currentPoints.empty();

		//extract last/first points if needed
		if (!noPoints){
			qplast = m_currentPoints.last();
			qpfirst = m_currentPoints.first();
		}

		// check if the new point is the first point, i.e the obstacle is closed
		qreal distance = (qp.rx()-qpfirst.rx())*(qp.rx()-qpfirst.rx()) 
			+ (qp.ry()-qpfirst.ry())*(qp.ry()-qpfirst.ry());

		if ((distance < PRECISION) && !noPoints)
		{
			if (obs)
				createObstacle();
			else 
				createRobot();
			clearLeftover();
			return true;
		}
		else
		{
			// Save current point
			m_currentPoints.push_back(QPointF(qp));
		
			// Add ellipse to the output
			QRectF rectangle = rectForPoint(qp,POINT_RADIUS);
			QGraphicsEllipseItem* ellipse = new QGraphicsEllipseItem( 0, this);;
			ellipse->setRect(rectangle);
			m_currentEllipses.push_back(ellipse);
			addItem(ellipse);

			if (!noPoints)
			{
				QLineF line(qplast,qp);
				QGraphicsLineItem*	lineItem = new QGraphicsLineItem(line,0,this);
				m_currentLines.push_back(lineItem);
				addItem(lineItem);
			}
			return false;
		}
	}

	bool addPointToRobot(QPointF qp)
	{
		return addPoint(qp,false);
	}

	/*	If the current obstacle has at least one point, it turns it into polygon, 
	otherwise it deletes it	*/
	void createRobot()
	{
		if (m_currentPoints.size() > 1)
		{
			m_current_robot++;
			QPolygonF polygon(m_currentPoints);

			QPointF ref_point = reference_point(polygon);

			m_robot_polygon.push_back(QPolygonF(m_currentPoints));
			m_robot_polygon[m_current_robot].translate(-ref_point.x(), -ref_point.y());
			addStartConfiguration();
		} 
	}

	static QPointF		reference_point(QPolygonF poly)
	{
		vector<pair<double,double> > pair_points;
		for (int i = 0; i < poly.size(); i++)
		{
			pair<double, double> pair_p(poly[i].x(), poly[i].y());
			pair_points.push_back(pair_p);
		}

		pair<double,double> ref_point_double = get_bounding_circle_center(pair_points.begin(), pair_points.end());
		QPointF ref(ref_point_double.first, ref_point_double.second);

		return ref;
	}

	void addConfiguration()
	{
		if (m_disp_conf_start[m_current_robot].size()
			==m_disp_conf_target[m_current_robot].size())
			addStartConfiguration();
		else
			addTargetConfiguration();
	}

	void addStartConfiguration()
	{
		QPolygonF polygon = m_robot_polygon[m_current_robot];
		// move polygon to center
		QPointF ref_point = polygon.first();
		QMatrix matrix_normalize;
		matrix_normalize.translate(-ref_point.rx(), -ref_point.ry());
		QPolygonF normalized_polygon = matrix_normalize.map(polygon);

		QGraphicsPolygonItem* item = new QGraphicsPolygonItem(0, this);
		item->setPolygon(normalized_polygon);
		if (m_disp_conf_start.size() < m_current_robot + 1)
		{
			m_disp_conf_start.push_back(QVector<QGraphicsPolygonItem*>());
			m_disp_conf_target.push_back(QVector<QGraphicsPolygonItem*>());
		}
		m_disp_conf_start[m_current_robot].push_back(item);

		item->setBrush(QBrush(GROUP_COLORS[m_current_robot]));
		item->setPolygon(polygon);
		item->setActive(true);
		item->setZValue(0.7);
		item->setFlag( QGraphicsItem::ItemIsMovable );
		item->setFlag( QGraphicsItem::ItemIsSelectable );
		if (m_conf_rotation_start.size() < m_current_robot + 1)
		{
			m_conf_rotation_start.push_back(QVector<int>());
			m_conf_rotation_target.push_back(QVector<int>());
		}
		m_conf_rotation_start[m_current_robot].push_back(0);		
		m_selected_configuration.first = m_current_robot;
		m_selected_configuration.second = m_conf_rotation_start[m_current_robot].size() - 1;
		m_selected_conf_type = false;
	}

	void addTargetConfiguration()
	{
		QPolygonF polygon = m_robot_polygon[m_current_robot];
		// move polygon to center
		QPointF ref_point = polygon.first();
		QMatrix matrix_normalize;
		matrix_normalize.translate(-ref_point.rx(), -ref_point.ry());
		QPolygonF normalized_polygon = matrix_normalize.map(polygon);

		QGraphicsPolygonItem* item = new QGraphicsPolygonItem(0, this);
		item->setPolygon(normalized_polygon);
		if (m_disp_conf_start.size() < m_current_robot + 1)
		{
			m_disp_conf_target.push_back(QVector<QGraphicsPolygonItem*>());
		}
		m_disp_conf_target[m_current_robot].push_back(item);

		//item->setBrush(QBrush(GROUP_COLORS[m_current_robot]));
		item->setPen(QPen(GROUP_COLORS[m_current_robot]));
	
		item->setPolygon(polygon);
		item->setActive(true);
		item->setZValue(0.7);
		item->setFlag( QGraphicsItem::ItemIsMovable );
		item->setFlag( QGraphicsItem::ItemIsSelectable );
		m_conf_rotation_target[m_current_robot].push_back(0);		
		m_selected_configuration.first = m_current_robot;
		m_selected_configuration.second = m_conf_rotation_target[m_current_robot].size() - 1;
		m_selected_conf_type = true;
	}

	void add_grid()
{
	double xl = -203; 
	double xr = 305;
	double xd = xr-xl;

	double yt = -252;
	double yb = 283;
	double yd = yb - yt;

	double step_size = 10;

	int x_num = xd / step_size;
	int y_num = yd / step_size;

	// vertical lines
	for (int i = 0; i < x_num; i++)
	{
		QPointF top(xl+i*step_size,yt);
		QPointF bot(xl+i*step_size,yb);
		QLineF line(top, bot);
		QGraphicsLineItem* item = new QGraphicsLineItem(0, this);
		if (i % 8 == 0)
			item->setPen(QPen(Qt::red));
		else if(i % 4 ==0)
			item->setPen(QPen(Qt::blue));
		else
			item->setPen(QPen(Qt::lightGray));
		item->setLine(line);
		item->setActive(true);
		item->setVisible(true);
		item->setZValue(0);
	}

	// horizontal lines
	for (int i = 0; i < x_num; i++)
	{
		QPointF top(xl,yt+i*step_size);
		QPointF bot(xr,yt+i*step_size);
		QLineF line(top, bot);
		QGraphicsLineItem* item = new QGraphicsLineItem(0, this);
		if (i % 8 == 0)
			item->setPen(QPen(Qt::red));
		else if(i % 4 ==0)
			item->setPen(QPen(Qt::blue));
		else
			item->setPen(QPen(Qt::lightGray));
		item->setLine(line);
		item->setActive(true);
		item->setVisible(true);
		item->setZValue(0);
	}
}

	/*	If the current obstacle has at least one point, it turns it into polygon, 
	otherwise it deletes it	*/
	void createObstacle()
	{
		if (m_currentPoints.size() > 1)
		{
			QPolygonF polygon(m_currentPoints);
			QGraphicsPolygonItem* item = new QGraphicsPolygonItem(0, this);
			m_vecObstacles.push_back(item);
			item->setBrush(QBrush(OBSTACLE_COLOR));
			item->setPolygon(polygon);
			item->setActive(true);
			item->setZValue(0.5);
			item->setFlag( QGraphicsItem::ItemIsMovable );
		} 
	}

	/*	Sets the room	*/
	void setRoom(QPointF topleft, QPointF bottomright);

	void animate();



	void rotateRobot(int diff)
	{
		int trueDegrees = diff / 8;
		int degree = trueDegrees / 2;

		rotateRobotDegrees(degree);
	}

	void rotateRobotDegrees(int deg)
	{
		int degree = deg;

		if (m_selected_conf_type == false)
		{
			QPolygonF robot_poly = 
				m_disp_conf_start[m_selected_configuration.first][m_selected_configuration.second]->polygon();

			QMatrix matrix_rotation;
			matrix_rotation.rotate(degree);

			m_conf_rotation_start[m_selected_configuration.first][m_selected_configuration.second] = 
				m_conf_rotation_start[m_selected_configuration.first][m_selected_configuration.second] + degree;
			QPolygonF rotated_polygon = matrix_rotation.map(robot_poly);

			m_disp_conf_start[m_selected_configuration.first][m_selected_configuration.second] ->setPolygon(rotated_polygon);
		}
		else
		{
			QPolygonF robot_poly = 
				m_disp_conf_target[m_selected_configuration.first][m_selected_configuration.second]->polygon();

			QMatrix matrix_rotation;
			matrix_rotation.rotate(degree);

			m_conf_rotation_target[m_selected_configuration.first][m_selected_configuration.second] = 
				m_conf_rotation_target[m_selected_configuration.first][m_selected_configuration.second] + degree;
			QPolygonF rotated_polygon = matrix_rotation.map(robot_poly);

			m_disp_conf_target[m_selected_configuration.first][m_selected_configuration.second] ->setPolygon(rotated_polygon);

		}
	}

	////////////////////////
	// Other methods
	////////////////////////
	
	///////////////////////
	// Helper methods
	///////////////////////

	/* Returns a rectangle with center point qp and height/width = radius*2 */
	QRectF rectForPoint(QPointF qp, double radius);

	void clearLeftover()
	{
		m_currentPoints.clear();
	
		while (!m_currentEllipses.empty())
		{
			QGraphicsEllipseItem* ellipse = m_currentEllipses.last();
			removeItem(ellipse);
			delete ellipse;
			m_currentEllipses.pop_back();
		}

		while (!m_currentLines.empty())
		{
			QGraphicsLineItem* line = m_currentLines.last();
			removeItem(line);
			delete line;
			m_currentLines.pop_back();
		}
	}


	QPointF point_normalize(QPointF p)
	{
		return QPointF(double_normalize_x(p.x()), double_normalize_y(p.y()));
	}

	double double_normalize_x(double x)
	{
		return 2 * (x + 218)/542 - 1.0;
	}
	
	double double_normalize_y(double y)
	{
		return 2 * (y + 253)/542 - 1.0;
	}

	QPointF point_unnormalize(QPointF p)
	{
		return QPointF(double_unnormalize_x(p.x()), double_unnormalize_y(p.y()));
	}

	double double_unnormalize_x(double x)
	{
		return (x + 1.0)*542/2 - 218;
	}
	
	double double_unnormalize_y(double y)
	{
		return (y + 1.0)*542/2  - 253;
	}

	static QPolygonF rotate_polygon(QPolygonF	p, double angle)
	{
		// TODO : fix this function to the new reference point
		QPolygonF polygon;
		QPointF ref_point = reference_point(p);

		QMatrix matrix_normalize;
		matrix_normalize.translate(-ref_point.rx(), -ref_point.ry());
		polygon = matrix_normalize.map(p);

		QMatrix matrix_rotation;
		matrix_rotation.rotate(angle);
		polygon = matrix_rotation.map(polygon);

		QMatrix matrix_unnormalize;
		matrix_unnormalize.translate(ref_point.rx(), ref_point.ry());
		polygon = matrix_unnormalize.map(polygon);

		return polygon;
	}

public slots:
	virtual void mousePressEvent( QMouseEvent *e ){};

	void selectedUpdate()
	{
		for (int r = 0; r < m_disp_conf_start.size(); r++)
		{
			for (int i = 0; i < m_disp_conf_start[r].size(); i++)
			{
				if (m_disp_conf_start[r][i]->isSelected())
				{
					m_selected_configuration.first = r;
					m_selected_configuration.second = i;
					m_selected_conf_type = false;
					m_current_robot = r;
					return;
				}
			}
		}

		for (int r = 0; r < m_disp_conf_target.size(); r++)
		{
			for (int i = 0; i < m_disp_conf_target[r].size(); i++)
			{
				if (m_disp_conf_target[r][i]->isSelected())
				{
					m_selected_configuration.first = r;
					m_selected_configuration.second = i;
					m_selected_conf_type = true;
					m_current_robot = r;
					return;
				}
			}
		}
	}

	void	save_robot(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}
		
		int num_robots = m_robot_polygon.size();
		fprintf(fOutput, "\n %d ", num_robots);

		for (int r = 0; r < num_robots; r++)
		{
			// move robot's reference point to (0,0);
			QPolygonF polygon  = m_robot_polygon[r];
			/*QPointF ref_point = polygon.first();
			QMatrix matrix;
			matrix.translate(-ref_point.rx(), -ref_point.ry());
			QPolygonF normalized_polygon = matrix.map(polygon);*/

			write_polygon(fOutput, polygon);
		}
		

		fclose(fOutput);
	}

	void	load_robot(const char* pi_szFileName)
	{
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		int num_robots;
		*in >> num_robots;
		
		m_current_robot = -1;

		for (int r = 0; r < num_robots; r++)
		{
			int num_vertices;
			*in >> num_vertices;
			m_currentPoints.clear();
			m_currentPoints =  read_polygon(num_vertices, in);
			createRobot();
		}
		
		ifile.close();
				m_currentPoints.clear();
	}

	void	save_workspace(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}
		int size = m_vecObstacles.size();
		fprintf(fOutput, "\n%d\n", size);

		for (int i = 0; i < size; i++)
		{
			QPolygonF fake_polygon = m_vecObstacles[i]->polygon();
			QPolygonF real_polygon = m_vecObstacles[i]->mapToScene(fake_polygon);
			write_polygon(fOutput, real_polygon);
		}

		fclose(fOutput);
	}

	void	load_workspace(const char* pi_szFileName)
	{
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		int num_obstacles;
		*in >> num_obstacles;

		if (num_obstacles != 0)
		{
			for (int i = 0; i < num_obstacles; i++)
			{
				int num_vertices;
				*in >> num_vertices;

				m_currentPoints =  read_polygon(num_vertices, in);
				createObstacle();
			}
		}
		
		ifile.close();
				m_currentPoints.clear();
	}

	QVector<QPointF>	read_polygon(int num_vertices, std::istream* in)
	{
		QVector<QPointF> result;
		for (int i = 0; i < num_vertices; i++)
		{
			double x,y;
			*in >> x >> y;
			result.push_back(point_unnormalize(QPointF(x,y)));
		}

		return result;
	}

	void	save_query(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}
		
		int robot_num = m_conf_rotation_start.size();
		fprintf(fOutput, "\n%d\n", robot_num);
		
		for (int r = 0; r < robot_num; r++)
		{
			int size = m_conf_rotation_start[r].size();
			fprintf(fOutput, "\n%d\n", size);

			for (int i = 0; i < size; i++)
			{
				QPointF fake_point = reference_point(m_disp_conf_start[r][i]->polygon());
				QPointF real_point = point_normalize(m_disp_conf_start[r][i]->mapToScene(fake_point));

				fprintf(fOutput, "%.2f %.2f %d ", real_point.x(), real_point.y(), m_conf_rotation_start[r][i]);
			}

			for (int i = 0; i < size; i++)
			{
				QPointF fake_point = reference_point(m_disp_conf_target[r][i]->polygon());
				QPointF real_point = point_normalize(m_disp_conf_target[r][i]->mapToScene(fake_point));

				fprintf(fOutput, "%.2f %.2f %d ", real_point.x(), real_point.y(), m_conf_rotation_target[r][i]);
			}
		}

		fclose(fOutput);
	}

	void	load_query(const char* pi_szFileName)
	{
		// TODO: fix 
		//const char* pi_szFileName2 = "D:\\Projects\\Software\\MRPoly\\MRMPApplication\\query.txt";
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		for (int r = 0; r < m_disp_conf_start.size(); r++)
		{
			for (int i = 0; i < m_disp_conf_start[r].size(); i++)
			{
				removeItem(m_disp_conf_start[r][i]);
			}
		}

		for (int r = 0; r < m_disp_conf_target.size(); r++)
		{
			for (int i = 0; i < m_disp_conf_target[r].size(); i++)
			{
				removeItem(m_disp_conf_target[r][i]);
			}
		}

		m_disp_conf_start.clear();
		m_conf_rotation_start.clear();

		m_disp_conf_target.clear();
		m_conf_rotation_target.clear();
		
		int num_robots;
		*in >> num_robots;

		for (int r = 0; r < num_robots; r++)
		{
			int num_confs;
			*in >> num_confs;

			for (int i = 0; i < num_confs; i++)
			{
				m_current_robot = r;
				addStartConfiguration();
				pair<int,int> conf = m_selected_configuration;
			
				double x,y;
				int theta;
				*in >> x >> y >> theta;
				QPointF point(x,y);
				QPointF scene_point = point_unnormalize(point);

				rotateRobotDegrees(theta);
				m_disp_conf_start[r][conf.second]->translate(scene_point.x(), scene_point.y());
			}

			for (int i = 0; i < num_confs; i++)
			{
				m_current_robot = r;
				addTargetConfiguration();
				pair<int,int> conf = m_selected_configuration;
			
				double x,y;
				int theta;
				*in >> x >> y >> theta;
				QPointF point(x,y);
				QPointF scene_point = point_unnormalize(point);

				rotateRobotDegrees(theta);
				m_disp_conf_target[r][conf.second]->translate(scene_point.x(), scene_point.y());
			}
		}
		
		ifile.close();

				m_currentPoints.clear();
	}

	void	load_path(const char* pi_szFileName)
	{	
		std::ifstream ifile(pi_szFileName);
		std::istream *in = &ifile; 

		m_path.clear();
		int num_groups;
		*in >> num_groups;
		vector<int> num_robots;

		for (int g = 0; g < num_groups; g++)
		{
			int r;
			* in >> r;
			num_robots.push_back(r);
		}

		int num_sections;
		*in >> num_sections;

		for (int i = 0; i < num_sections; i++)
		{
			vector<vector<QConf> > current_section(num_groups);

			for (int g = 0; g < num_groups; g++)
			{
				for (int r = 0; r < num_robots[g]; r++)
				{
					double x,y;
					int theta;
					*in >> x >> y >> theta;
					QPointF point(x,y);
					//QPointF scene_point = point_unnormalize(point);
					QConf conf(point, theta);
					current_section[g].push_back(conf);
				}
			}

			m_path.push_back(current_section);
		}

		ifile.close();
		m_currentPoints.clear();
	}

	void	save_path(const char* pi_szFileName)
	{
		FILE* fOutput = fopen(pi_szFileName, "w+");
		if (!fOutput)
		{
			return;
		}
		int num_groups = m_disp_conf_start.size();
		fprintf(fOutput, "\n%d\n", num_groups);
		vector<int> num_robots;
		for (int g = 0; g < num_groups; g++)
		{
			int r = m_disp_conf_start[g].size();
			num_robots.push_back(r);
			fprintf(fOutput, "%d ", r);
		}

		int sections_num = m_path.size();
		
		fprintf(fOutput, "\n%d\n", sections_num);
		for (int i = 0; i < sections_num; i++)
		{
			for (int g = 0; g < num_groups; g++)
			{
				for (int r = 0; r < num_robots[g]; r++)
				{
					QConf conf = m_path[i][g][r];
					fprintf(fOutput, " %.5f %.5f %d ", conf.first.x(), conf.first.y(), conf.second);
				}
			}
		}

		fclose(fOutput);
	}

	void animationComplete()
	{
		clear_animation();
		//change_conf_state();
	}

	void write_polygon(FILE* file, QPolygonF poly)
	{
		int num_vertices = poly.size();
		fprintf(file, "\n %d ", num_vertices);

		for (int i = 0; i < num_vertices; i++)
		{
			QPointF norm_point = point_normalize(poly[i]);
			double dx = norm_point.rx();
			double dy = norm_point.ry();

			fprintf(file, "%.2f %.2f ", dx, dy);
		}
	}

	void	clear_animation()
{
	for (int g = 0; g < m_displayed_animation_objects.size(); g++)
	{
		for (int i = 0; i < m_displayed_animation_objects[g].size(); i++)
		{
			removeItem(m_displayed_animation_objects[g][i]);
		}
	}
	
	m_displayed_animation_objects.clear();
}

/*
	The following functions are used by the KPUMP algorithm
*/

	QVector<QPolygonF>	getObstacles()
	{
		QVector<QPolygonF> res;
		for (int i=0; i<m_vecObstacles.size(); i++)
		{
			QPolygonF poly = m_vecObstacles[i]->polygon();
			res.push_back(poly);
		}
		return res;
	}

	QPointF getRoomTopLeft()
	{
		return m_roomTL;
	}
	
	QPointF getRoomBottomRight()
	{
		return m_roomBR;
	}

	QVector<QVector<QPointF> >	getStartPositions()
	{
		int robot_num = m_conf_rotation_start.size();
		QVector<QVector<QPointF> > result(robot_num);
		
		for (int r = 0; r < robot_num; r++)
		{
			int size = m_conf_rotation_start[r].size();

			for (int i = 0; i < size; i++)
			{
				QPointF fake_point = reference_point(m_disp_conf_start[r][i]->polygon());
				QPointF real_point = m_disp_conf_start[r][i]->mapToScene(fake_point);
				result[r].push_back(real_point);
			}
		}
		return result;
	}
	
	QVector<QVector<QPointF> >	getTargetPositions()
	{
		int robot_num = m_conf_rotation_start.size();
		QVector<QVector<QPointF> > result(robot_num);
		
		for (int r = 0; r < robot_num; r++)
		{
			int size = m_conf_rotation_start[r].size();

			for (int i = 0; i < size; i++)
			{
				QPointF fake_point = reference_point(m_disp_conf_target[r][i]->polygon());
				QPointF real_point = m_disp_conf_target[r][i]->mapToScene(fake_point);
				result[r].push_back(real_point);
			}
		}
		return result;
	}

	vector<QPolygonF>	getRobotPolygons()
	{
		return m_robot_polygon;
	}

	void				setPath(vector<vector<vector<QConf> > > path)
	{
		m_path = path;

	}


signals: 
	void displayChanged();

	void animationDone();

protected:
	
	virtual void keyPressEvent (QKeyEvent* evt);

	////////////////////////
	// Data Members
	////////////////////////
public:
	
	//	obstacles information
	QVector<QPointF>				m_currentPoints;
	QVector<QGraphicsEllipseItem*>	m_currentEllipses;
	QVector<QGraphicsLineItem*>		m_currentLines;
	QVector<QGraphicsPolygonItem*>	m_vecObstacles;

	//	robots information
	vector<QPolygonF>							m_robot_polygon;
	vector<QVector<QGraphicsPolygonItem*> >		m_disp_conf_start;
	vector<QVector<QGraphicsPolygonItem*> >		m_disp_conf_target;
	QVector<QVector<QGraphicsPolygonItem*> >		m_displayed_animation_objects;
	QVector<QVector<int> >						m_conf_rotation_start;
	QVector<QVector<int> >						m_conf_rotation_target;
	pair<int,int>								m_selected_configuration;
	bool										m_selected_conf_type; // start(0) or target(1)
	int											m_current_robot;

	QPointF							m_roomTL;
	QPointF							m_roomBR;

	// path section - group - robot
	vector<vector<vector<QConf> > >			m_path;
};

#endif
