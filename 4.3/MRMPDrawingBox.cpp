/*******************************************************************
*	File name: 		MRMPDrawingBox.cpp
*	Description:	Implementation of the MRMPDrawingBox class.
*					This class is a component of the GUI main window.
*					It inherits QGraphicsView and it is used as the drawing area of the application.
*	Author:			Kiril Solovey
*
*	Date:			29/3/2011
*******************************************************************/

#include "MRMPDrawingBox.h"
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QRect>
#include <QRectF>
#include <QGraphicsView>
#include <QPolygonF>
#include <QPropertyAnimation>
#include <QGraphicsPolygonItem>
#include <string>
#include <QPolygonF>
#include <QMessageBox>
#include <QMetaType>


Q_DECLARE_METATYPE(QPolygonF)

#pragma warning(disable: 4291 4503)
using namespace std;

const int DEFAULT_ANIMATION_SPEED = 500;

MRMPDrawingBox::MRMPDrawingBox(QWidget *parent)
	: QGraphicsView(parent)
{
	ui.setupUi(this);
	m_currentState = IDLE;
	m_mpScene=new Scene(this);
	this->setScene( m_mpScene );
	m_mpScene->setSceneRect(this->viewport()->geometry());
	QRectF rect = this->viewport()->geometry();
	QObject::connect(m_mpScene,SIGNAL(displayChanged()),this,SLOT(refresh()));
	//QObject::connect(m_mpScene,SIGNAL(animationDone()),this,SLOT(animationComplete()));

	//m_mpScene->animate();
	refresh();
	m_socket_counter = 0;

	setRenderHints(QPainter::Antialiasing);
}



MRMPDrawingBox::~MRMPDrawingBox()
{
	clear();
}

void MRMPDrawingBox::animationComplete()
{

	m_currentState = IDLE;

	//m_mpScene ->clear_animation();

	refresh();

}

//when our widget is pressed, and we are in a drawing modes we record point.
//and draw it by updating the widget.
void MRMPDrawingBox::mousePressEvent( QMouseEvent *e )
{
	
	QPointF sPos = mapToScene(e->posF().toPoint());
	setRenderHints(QPainter::Antialiasing);
	if (m_currentState == DRAW_OBSTACLE)
	{
		bool bres = m_mpScene->addPointToObstacle(sPos);
		if (bres)
			m_currentState = IDLE;	
	} 
	else if (m_currentState == DRAW_ROBOT)
	{
		bool bres = m_mpScene->addPointToRobot(sPos);
		if (bres)
			m_currentState = IDLE;	
		/*m_mpScene->addRobot(sPos);
		m_currentState = DRAW_TARGET;*/
	} else if (m_currentState == DRAW_TARGET)
	{
		/*m_mpScene->addTarget(sPos);
		m_currentState = IDLE;*/
	}

	this->show();	
}

//	tracks mouse wheel events
/*void MRMPDrawingBox::wheelEvent ( QWheelEvent *e )
{
	int diff = e->delta();
	m_mpScene->rotateRobot(diff);
	this->show();
}*/



//	orders the animation of paths
void MRMPDrawingBox::animateButtonPressed()
{
	m_mpScene->animate();
	this->show();
}

template<typename OutputIterator>
		void string_split(std::string str,
						 std::string delim,
						 OutputIterator& oi)
		{
		 int cut_at;

		 while( (cut_at = str.find_first_of(delim)) != str.npos )
		 {
		   if(cut_at > 0)
		   {
			 std::string curr(str.substr(0, cut_at));
			 *oi++ = curr;
		   }
		   str = str.substr(cut_at+1);
		 }

		 if(str.length() > 0)
		   *oi++ = str;

		 return;

	}


QPolygonF MRMPDrawingBox::converToSceneCords(QPolygonF& poly)
{
	QPolygonF f;
	return f;
}

//start recording points for a new obstacle.
void MRMPDrawingBox::drawObstaclesButtonPushed(){
	if (m_currentState == IDLE)
		m_currentState = DRAW_OBSTACLE;
}

//start recording points for a new robot.
void MRMPDrawingBox::drawRobotsButtonPushed(){
	m_currentState = DRAW_ROBOT;
}

void MRMPDrawingBox::clear(){
	this->update();
	this->show();
}
void MRMPDrawingBox::clearResults(){

	this->update();
	this->show();
}

void MRMPDrawingBox::setSceneRoom(){
		QVector<QPointF> pointVector;
		QRect windGeo=this->viewport()->geometry();

		m_mpScene->setRoom(this->mapToScene(windGeo.topLeft()),
			this->mapToScene(windGeo.bottomRight()));
}

// Invokes the motion planning algorithm
void MRMPDrawingBox::execute()
{
	setSceneRoom();
	vector<vector<Conf> > retPath;
	double quality;
	Planner planner(m_mpScene, 45, false, 0, &retPath, &quality);
	planner.run();
}
