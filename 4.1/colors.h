#ifndef COLORS_H
#define COLORS_H

#include <qcolor.h> 
#include <QBrush>
#include <QLinearGradient>
#include <vector>

const int NUM_COLORS = 10;

const QColor GROUP_COLORS[] = {
  Qt::blue,
  Qt::red, Qt::magenta, Qt::green, 
  Qt::yellow, Qt::cyan, Qt::darkBlue, 
  Qt::darkGreen, Qt::darkRed, Qt::darkYellow};

const QColor	ROBOT_STYLE		= Qt::SolidPattern;

const Qt::BrushStyle	TARGET_STYLE	= Qt::DiagCrossPattern;

const QColor	OBSTACLE_COLOR	= Qt::lightGray;

const QColor 	ROBOT_COLOR[] = {Qt::blue, Qt::red};

const QColor	CONFIGURATION_COLOR = Qt::white;

const QColor	ANIMATED_ROBOT_COLOR[] = {Qt::cyan, Qt::green};

const QColor	CONFIGURATION_COLOR_TOP[] = {Qt::white, Qt::blue, Qt::white, Qt::blue};

const QColor	CONFIGURATION_COLOR_BOTTOM[] = {Qt::white, Qt::white, Qt::red, Qt::red};

const Qt::BrushStyle	OBSTACLE_STYLE	= Qt::SolidPattern;

#endif