#ifndef VORONOI_H
#define VORONOI_H

#include "utils.hpp"

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "boost/polygon/voronoi.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <utility>

namespace student {
    struct VorPoint {
        int x;
        int y;

        VorPoint(int a, int b) : x(a), y(b) {};

        VorPoint(Point p) : x(p.x), y(p.y) {};
    };

    struct VorSegment {
        VorPoint p0;
        VorPoint p1;

        VorSegment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {};

        typedef int coordinate_type;
        typedef student::VorPoint point_type;

        VorPoint get(boost::polygon::direction_1d &dir) const {
            return dir.to_int() ? this->p1 : this->p0;
        }
    };

    namespace bg = boost::geometry;
    namespace bp = boost::polygon;

    typedef bg::model::d2::point_xy<double> point_type_def;
    typedef bg::model::polygon <point_type_def> polygon_type_def;

    typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
    typedef boost::adjacency_list <boost::listS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> UndirectedGraph;
    typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
    typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor vertex_descriptor;

    std::vector <VorSegment> mapPolygonsToVoronoiSegments(const std::vector <Polygon> &obstaclesAndBorders);

    int findVertexIndex(const std::vector <bp::voronoi_vertex<double>> &vertices, const double x, const double y);

    double computeDistance(const std::vector <bp::voronoi_vertex<double>> &vertices, const int idx0, const int idx1);

    std::pair <UndirectedGraph, std::vector<bp::voronoi_vertex < double>>>

    getVoronoiGraph(const std::vector <Polygon> &obstacles, const Polygon &borders, const Point start, const Point end);
} // namespace student

namespace boost {
    namespace polygon {
        template<>
        struct geometry_concept<student::VorPoint> {
            typedef point_concept type;
        };

        template<>
        struct point_traits<student::VorPoint> {
            typedef int coordinate_type;

            static inline coordinate_type get(const student::VorPoint &point, orientation_2d orient) {
                return (orient == HORIZONTAL) ? point.x : point.y;
            }
        };

        template<>
        struct geometry_concept<student::VorSegment> {
            typedef segment_concept type;
        };

        template<>
        struct point_traits<student::VorSegment> {
            typedef int coordinate_type;
            typedef student::VorPoint point_type;

            static inline point_type get(const student::VorSegment &segment, direction_1d dir) {
                return dir.to_int() ? segment.p1 : segment.p0;
            }
        };

    } // namespace polygon
} // namespace boost

#endif
