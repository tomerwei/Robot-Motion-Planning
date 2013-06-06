#ifndef BASIC_TYPEDEF_H
#define BASIC_TYPEDEF_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_default_dcel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Cartesian_d.h>

#include <vector>
#include <map>
#include <list>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/foreach.hpp>

#include <QVector>
#include <QPointF>

#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_default_dcel.h>

#include <CGAL/Arr_trapezoid_ric_point_location.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Arr_walk_along_line_point_location.h>

//#define foreach         BOOST_FOREACH

//#pragma warning ( disable : 4819 )

using namespace std;

/*******************************************************************************************
 * This file contatins basic typedefs (from CGAL and more).
 *******************************************************************************************/

typedef CGAL::Exact_predicates_exact_constructions_kernel				Kernel;
typedef Kernel::Point_2													Point_2;
typedef	std::vector<Point_2>											Container;

typedef Kernel::Vector_2							Vector_2;
typedef Kernel::Segment_2							Segment_2;
typedef CGAL::Polygon_set_2<Kernel>					Polygon_set_2;
typedef Polygon_set_2::Arrangement_2                Arrangement_2;
typedef Polygon_set_2::Polygon_2					Polygon_2;
typedef Polygon_set_2::Polygon_with_holes_2			Polygon_with_holes_2;

typedef	CGAL::Arr_trapezoid_ric_point_location<Arrangement_2>		Ric_pl;

typedef Point_2										Conf;
typedef vector<Conf>								ConfSet;
typedef list<Polygon_2>								Obstacles;
typedef pair<Point_2,Point_2>						Room;

typedef pair<QPointF,double>						QConf;

// d-dimensional kernel and point
typedef CGAL::Cartesian_d<double> Kernel_d;
typedef Kernel_d::Point_d Point_d;

typedef CGAL::Exact_predicates_inexact_constructions_kernel	Kernel_ie;
typedef Kernel_ie::Point_2					Point_2_ie;
typedef Kernel_ie::Vector_2					Vector_2_ie;

#endif