#include "planning.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

namespace student {
    std::vector <VorSegment> mapPolygonsToVoronoiSegments(const std::vector <Polygon> &obstaclesAndBorders) {
        std::vector <VorSegment> segments;
        for (const Polygon &obstacle : obstaclesAndBorders) {
            VorPoint start = VorPoint(obstacle[0]);
            for (int i = 1; i < obstacle.size(); i++) {
                const VorPoint end = VorPoint(obstacle[i]);
                segments.emplace_back(start.x, start.y, end.x, end.y);
                start = end;
            }

            // Add last point to close the polygon
            const VorPoint end = VorPoint(obstacle[0]);
            segments.emplace_back(start.x, start.y, end.x, end.y);
        }
        return segments;
    }

    int findVertexIndex(const std::vector <bp::voronoi_vertex<double>> &vertices, const double x, const double y) {
        auto iterator = std::find_if(vertices.begin(), vertices.end(),
                                     [&](bp::voronoi_vertex<double> el) {
                                         return (x == el.x() && y == el.y());
                                     });
        auto idx = std::distance(vertices.begin(), iterator);
        return idx;
    }

    double computeDistance(const std::vector <bp::voronoi_vertex<double>> &vertices, const int idx0, const int idx1) {
        point_type_def p0(vertices[idx0].x(), vertices[idx0].y());
        point_type_def p1(vertices[idx1].x(), vertices[idx1].y());
        return bg::distance(p0, p1);
    }

    bool isSegmentInFigure(const bp::voronoi_vertex<double> &vertex0, const bp::voronoi_vertex<double> &vertex1,
                           const std::vector <Polygon> &figures) {
        point_type_def vPoint0(vertex0.x(), vertex0.y());
        point_type_def vPoint1(vertex1.x(), vertex1.y());

        for (auto figure : figures) {
            polygon_type_def poly;
            for (auto point : figure) {
                point_type_def p(point.x, point.y);
                bg::append(poly.outer(), p);
            }
            point_type_def p(figure[0].x, figure[0].y);
            bg::append(poly.outer(), p);

            if (bg::within(vPoint0, poly) || bg::within(vPoint1, poly))
                return true;
        }
        return false;
    }

    std::pair <UndirectedGraph, std::vector<bp::voronoi_vertex < double>>>

    getVoronoiGraph(const std::vector <Polygon> &obstacles, const Polygon &borders, const Point start,
                    const Point end) {
        std::vector <VorPoint> newPoints;
        newPoints.push_back(VorPoint(start));
        newPoints.push_back(VorPoint(end));

        std::vector <Polygon> obstaclesAndBorders(obstacles);
        obstaclesAndBorders.push_back(borders);

        std::vector <VorSegment> segments = mapPolygonsToVoronoiSegments(obstaclesAndBorders);

        voronoi_diagram<double> vd;
        construct_voronoi(newPoints.begin(), newPoints.end(), segments.begin(), segments.end(), &vd);

        auto vertices = vd.vertices();
        vertices.push_back(bp::voronoi_vertex<double>(start.x, start.y));
        vertices.push_back(bp::voronoi_vertex<double>(end.x, end.y));

        UndirectedGraph g;

        point_type_def startPoint(start.x, start.y);
        point_type_def endPoint(end.x, end.y);

        auto startIdx = findVertexIndex(vertices, start.x, start.y);
        auto endIdx = findVertexIndex(vertices, end.x, end.y);
        for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
             it != vd.cells().end(); ++it) {
            const voronoi_diagram<double>::cell_type &cell = *it;
            const voronoi_diagram<double>::edge_type *edge = cell.incident_edge();

            polygon_type_def poly;
            point_type_def firstPoint;
            bool first = true;

            do {
                if (edge->is_primary() && edge->vertex0() && edge->vertex1()) {
                    if (!isSegmentInFigure(*edge->vertex0(), *edge->vertex1(), obstacles)) {
                        auto firstIdx = findVertexIndex(vertices, edge->vertex0()->x(), edge->vertex0()->y());
                        auto secondIdx = findVertexIndex(vertices, edge->vertex1()->x(), edge->vertex1()->y());

                        boost::add_edge(firstIdx, secondIdx, computeDistance(vertices, firstIdx, secondIdx), g);
                    }

                    point_type_def p(edge->vertex0()->x(), edge->vertex0()->y());
                    bg::append(poly.outer(), p);

                    if (first == true) {
                        firstPoint = p;
                        first = false;
                    }
                }
                edge = edge->next();
            } while (edge != cell.incident_edge());

            bg::append(poly.outer(), firstPoint);

            bool isInStart = bg::within(startPoint, poly);
            bool isInEnd = bg::within(endPoint, poly);

            if (isInStart || isInEnd) {
                for (auto it = boost::begin(bg::exterior_ring(poly)); it != boost::end(bg::exterior_ring(poly)); ++it) {
                    double x = bg::get<0>(*it);
                    double y = bg::get<1>(*it);

                    auto firstIdx = findVertexIndex(vertices, x, y);
                    if (isInStart)
                        boost::add_edge(firstIdx, startIdx, computeDistance(vertices, firstIdx, startIdx), g);
                    else if (isInEnd)
                        boost::add_edge(firstIdx, endIdx, computeDistance(vertices, firstIdx, endIdx), g);
                }
            }
        }

        return std::make_pair(g, vertices);
    }
} // namespace student
