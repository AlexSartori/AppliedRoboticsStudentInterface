#include "planning.hpp"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

namespace student {

    /*!
     * Convert a vector of polygons to a vector of Voronoi segments
     * @param obstaclesAndBorders The given Polygons to be converted
     * @return The resulting vector of Voronoi segments
     */
    std::vector <VorSegment> mapPolygonsToVoronoiSegments(const std::vector <Polygon> &obstaclesAndBorders) {
        std::vector <VorSegment> segments;
        for (const Polygon &obstacle : obstaclesAndBorders) {
            if(obstacle.size() == 0)
                continue;

            VorPoint start(obstacle[0]);
            for (int i = 1; i < obstacle.size(); i++) {
                VorPoint end(obstacle[i]);
                segments.emplace_back(start.x, start.y, end.x, end.y);
                start = end;
            }

            // Add last point to close the polygon
            VorPoint end(obstacle[0]);
            segments.emplace_back(start.x, start.y, end.x, end.y);
        }
        return segments;
    }

    /*!
     * Find the index of a vertex inside a Voronoi graph
     * @param vertices The vector of vertices
     * @param x The x of the point ot find
     * @param y The y of the point to find
     * @return The index of the vertex
     */
    int findVertexIndex(const std::vector <bp::voronoi_vertex<double>> &vertices, const double x, const double y) {
        auto iterator = std::find_if(vertices.begin(), vertices.end(),
                                     [&](bp::voronoi_vertex<double> el) {
                                         return (x == el.x() && y == el.y());
                                     });
        auto idx = std::distance(vertices.begin(), iterator);
        return idx;
    }

    /*!
     * Compute the distance between two vertices
     * @param vertices The vector containing the vertices
     * @param idx0 The index of the first vertex
     * @param idx1 The index of the second vertex
     * @return The distance between the two vertices
     */
    double computeDistance(const std::vector <bp::voronoi_vertex<double>> &vertices, const int idx0, const int idx1) {
        point_type_def p0(vertices[idx0].x(), vertices[idx0].y());
        point_type_def p1(vertices[idx1].x(), vertices[idx1].y());
        return bg::distance(p0, p1);
    }

    /*!
     * Check if a given segment across anyone of the given polygons
     * @param vertex0 The first vertex of the segment
     * @param vertex1 The second vertex of the segment
     * @param figures The vector containing the polygons
     * @return `true` if at least one polygon get crossed by the segment, `false` otherwise
     */
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

    /*!
     * Create a polygon around a given point
     * @param p The point
     * @return The constructed polygon
     */
    Polygon createPolygonAroundPoint(const Point &p) {
        const int length = 20;
        Polygon poly;
        poly.emplace_back(p.x + length, p.y - length);
        poly.emplace_back(p.x + length, p.y);
        poly.emplace_back(p.x + length, p.y + length);
        poly.emplace_back(p.x, p.y + length);
        poly.emplace_back(p.x - length, p.y + length);
        poly.emplace_back(p.x - length, p.y);
        poly.emplace_back(p.x - length, p.y - length);
        poly.emplace_back(p.x, p.y - length);

        return poly;
    }

    /*!
     * Save the image showing the Voronoi diagram
     * @param vd The Voronoi diagram
     * @param start The starting position
     * @param end The ending position
     */
    void drawVoronoiImage(const voronoi_diagram<double> &vd, const Point &start, const Point &end) {
        cv::Mat tmp(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::Mat tmpPrim(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        int thickness = 2;
        int lineType = cv::LINE_8;

        for (auto it = vd.edges().begin(); it != vd.edges().end(); ++it) {
            if (it->vertex0() && it->vertex1()) {
                cv::Point cvstart = cv::Point(it->vertex0()->x(), it->vertex0()->y());
                cv::Point cvend = cv::Point(it->vertex1()->x(), it->vertex1()->y());
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 255, 0), thickness, lineType);
                if(it->is_primary()){
                    cv::line(tmpPrim, cvstart, cvend, cv::Scalar(0, 255, 0), thickness, lineType);
                }
            }
        }

        cv::circle(tmp, cv::Point(start.x, start.y), 5, cv::Scalar(255, 0, 0), CV_FILLED, 8, 0);
        cv::circle(tmp, cv::Point(end.x, end.y), 5, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

        auto pathImg = "/root/workspace/Voronoi.png";
        auto pathImgPrim = "/root/workspace/Voronoi_prim.png";
        //cv::imwrite(pathImg, tmp);
        //cv::imwrite(pathImgPrim, tmpPrim);

        //std::cout << "img saved at "<<pathImg << std::endl;
        /*cv::namedWindow("Path Voronoi", cv::WINDOW_NORMAL);
        cv::imshow("Path Voronoi", tmp);
        cv::waitKey(0);*/
    }

    /*!
     * Check if the two given point (a Voronoi vertex and a Boost point) are near enough to state that they are the same point
     * @param a The Voronoi vertex
     * @param b The Boost point
     * @param tolerance The tolerance to use while comparing them
     * @return `true` if the two points are close enough, `false` otherwise
     */
    bool isTheSamePoint(const bp::voronoi_vertex<double> &a, const point_type_def & b, const int tolerance) {
        return ((a.x() < b.x() + tolerance && a.x() > b.x() - tolerance) &&
                (a.y() < b.y() + tolerance && a.y() > b.y() - tolerance));
    }

    /*!
     * Construct a Voronoi graph from the arena and extract a graph from it
     * @param obstacles The obstacles of the arena
     * @param borders The borders of the arena
     * @param start The starting position of the robot
     * @param end the ending position of the robot
     * @return The resulting graph
     */
    std::pair <UndirectedGraph, std::vector<bp::voronoi_vertex < double>>>

    getVoronoiGraph(const std::vector <Polygon> &obstacles, const Polygon &borders, const Point start,
                    const Point end) {
        std::vector <VorPoint> newPoints;

        std::vector <Polygon> obstaclesAndBorders(obstacles);
        obstaclesAndBorders.push_back(borders);
        // for the voronoi construction use even some polygons constructed around the target points
        obstaclesAndBorders.push_back(createPolygonAroundPoint(start));
        obstaclesAndBorders.push_back(createPolygonAroundPoint(end));

        std::vector <VorSegment> segments = mapPolygonsToVoronoiSegments(obstaclesAndBorders);

        voronoi_diagram<double> vd;
        construct_voronoi(newPoints.begin(), newPoints.end(), segments.begin(), segments.end(), &vd);

        auto vertices = vd.vertices();
        // add the target points as vertices for the future graph
        vertices.push_back(bp::voronoi_vertex<double>(start.x, start.y));
        vertices.push_back(bp::voronoi_vertex<double>(end.x, end.y));

        drawVoronoiImage(vd, start, end);

        UndirectedGraph g;

        point_type_def startPoint(start.x, start.y);
        point_type_def endPoint(end.x, end.y);

        auto startIdx = findVertexIndex(vertices, start.x, start.y);
        auto endIdx = findVertexIndex(vertices, end.x, end.y);

        for (auto it = vd.edges().begin(); it != vd.edges().end(); ++it) {
            if (it->vertex0() && it->vertex1()){
                if (!isSegmentInFigure(*it->vertex0(), *it->vertex1(), obstacles)) {
                    auto firstIdx = findVertexIndex(vertices, it->vertex0()->x(), it->vertex0()->y());
                    auto secondIdx = findVertexIndex(vertices, it->vertex1()->x(), it->vertex1()->y());

                    // map the center of the previous constructed polygons as the target points (close enough points)
                    const int tolerance = 2;
                    if(isTheSamePoint(*it->vertex0(), startPoint, tolerance))
                        firstIdx = startIdx;
                    if(isTheSamePoint(*it->vertex1(), startPoint, tolerance))
                        secondIdx = startIdx;
                    if(isTheSamePoint(*it->vertex0(), endPoint, tolerance))
                        firstIdx = endIdx;
                    if(isTheSamePoint(*it->vertex1(), endPoint, tolerance))
                        secondIdx = endIdx;

                    boost::add_edge(firstIdx, secondIdx, computeDistance(vertices, firstIdx, secondIdx), g);
                }
            }
        }

        return std::make_pair(g, vertices);
    }
}
