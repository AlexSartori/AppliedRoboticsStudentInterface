#include "planning.hpp"

namespace student {
    bool victimSortFn (const std::pair<int, Polygon>& struct1, const std::pair<int, Polygon>& struct2) {
        return (struct1.first < struct2.first);
    }

    Polygon scaleUpPolygon(const Polygon &poly) {
        Polygon newPoly;
        for (int i = 0; i < poly.size(); i++) {
            Point p(poly[i]);
            p.x *= OBJECTS_SCALE_FACTOR;
            p.y *= OBJECTS_SCALE_FACTOR;
            newPoly.push_back(p);
        }
        return newPoly;
    }
    
    void expandPolygon(Polygon &polygon) {
        ClipperLib::Path p;
        ClipperLib::Paths solution;

        for (Point pt : polygon){
            p << ClipperLib::IntPoint(pt.x * OBJECTS_SCALE_FACTOR, pt.y * OBJECTS_SCALE_FACTOR);
        }
        
        ClipperLib::ClipperOffset co;
        co.AddPath(p, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(solution, 15 * OBJECTS_SCALE_FACTOR);

        polygon.clear();
        for (ClipperLib::Path sol : solution)
            for (ClipperLib::IntPoint pt : sol)
                polygon.emplace_back(pt.X / OBJECTS_SCALE_FACTOR, pt.Y / OBJECTS_SCALE_FACTOR);
    }

    void joinOverlappingPolygons(std::vector<Polygon> &polygons) {
        for(int i = 0; i < polygons.size(); i++) {
            for(int j = 0; j < polygons.size(); j++) {
                if (i == j || polygons[i].size() == 0 || polygons[j].size() == 0)
                    continue;
                ClipperLib::Paths subj(1), clip(1), solution;

                for (Point pt : polygons[i])
                    subj[0] << ClipperLib::IntPoint(pt.x * OBJECTS_SCALE_FACTOR, pt.y * OBJECTS_SCALE_FACTOR);
                for (Point pt : polygons[j])
                    clip[0] << ClipperLib::IntPoint(pt.x * OBJECTS_SCALE_FACTOR, pt.y * OBJECTS_SCALE_FACTOR);

                ClipperLib::Clipper c;
                c.AddPaths(subj, ClipperLib::ptSubject, true);
                c.AddPaths(clip, ClipperLib::ptClip, true);
                c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

                if(solution.size() == 1) {
                    polygons[i].clear();
                    polygons[j].clear();
                    for (ClipperLib::IntPoint pt : solution[0])
                        polygons[i].emplace_back(pt.X / OBJECTS_SCALE_FACTOR, pt.Y / OBJECTS_SCALE_FACTOR);
                }
            }
        }

        for (auto it = polygons.begin(); it != polygons.end();) {
            if((*it).size() == 0)
                polygons.erase(it);
            else
                ++it;
        }
    }

    polygon_type_def convertPolygonToBoost(const Polygon &input) {
        polygon_type_def poly;
        point_type_def firstPoint;

        for(auto &v : input){
            point_type_def tmpP(v.x, v.y);
            bg::append(poly.outer(), tmpP);
        }
        point_type_def tmpP(input[0].x, input[0].y);
        bg::append(poly.outer(), tmpP);

        return poly;
    }

    void drawGraphImage(const UndirectedGraph &graph, const std::vector<bp::voronoi_vertex<double>> &vertices,
                        const Point &currPosition, const Point &targetPoint, const std::vector <Polygon> &obstacles) {
        // let's draw the result
        cv::Mat tmp(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        int thickness = 2;
        int lineType = cv::LINE_8;

        std::pair <edge_iterator, edge_iterator> edgePair;
        for (edgePair = edges(graph); edgePair.first != edgePair.second; ++edgePair.first) {
            auto a = vertices[source(*edgePair.first, graph)];
            auto b = vertices[target(*edgePair.first, graph)];

            cv::Point cvstart = cv::Point(a.x(), a.y());
            cv::Point cvend = cv::Point(b.x(), b.y());
            cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 0), thickness, lineType);
        }

        for (auto &obst : obstacles){
            for (int i = 1; i <= obst.size(); i++){
                cv::Point cvstart = cv::Point(obst[i % obst.size()].x, obst[i % obst.size()].y);
                cv::Point cvend = cv::Point(obst[i-1].x, obst[i-1].y);
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 255), thickness, lineType);
            }
        }

        cv::circle(tmp, cv::Point(currPosition.x, currPosition.y), 5, cv::Scalar(255, 0, 0), CV_FILLED, 8, 0);
        cv::circle(tmp, cv::Point(targetPoint.x, targetPoint.y), 5, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

        auto pathImg = "/root/workspace/Graph.png";
        cv::imwrite(pathImg, tmp);
        //std::cout << "img saved at "<<pathImg << std::endl;

        
        //cv::namedWindow("Path Voronoi", cv::WINDOW_NORMAL);
        //cv::imshow("Path Voronoi", tmp);
        //cv::waitKey(0);
    }

    bool elaborateVoronoi(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                          const std::vector <std::pair<int, Polygon>> &victim_list,
                          const Polygon &gate, const Point &robot, std::vector<Point> &pointPath) {

        Point currPosition(robot);
        Polygon prevTarget;

        std::vector<Polygon> targets;
        std::vector <std::pair<int, Polygon>> sortedVictims(victim_list);

        // create a target array sorted by priority
        std::sort (sortedVictims.begin(), sortedVictims.end(), victimSortFn);
        for(auto &v : sortedVictims)
            targets.push_back(v.second);
        targets.push_back(gate);

        // an array of all the possible objects present in the arena
        std::vector <Polygon> allObjects;
        for (Polygon p : obstacle_list)
            allObjects.push_back(p);

        for (auto victim : victim_list)
            allObjects.push_back(victim.second);
        allObjects.push_back(gate);

        for(auto &targetPoly : targets) {
            // array of objects that the robot should avoid to go on
            std::vector <Polygon> toAvoid(allObjects);
            toAvoid.erase(std::remove(toAvoid.begin(), toAvoid.end(), targetPoly));
            if (prevTarget.size() > 0)
                toAvoid.erase(std::remove(toAvoid.begin(), toAvoid.end(), prevTarget));

            for (int i = 0; i < toAvoid.size(); i++) {
                expandPolygon(toAvoid[i]);
            }
            // merge together overlapping obstacles
            joinOverlappingPolygons(toAvoid);

            polygon_type_def poly = convertPolygonToBoost(targetPoly);

            // get the centroid of the targetPoly to use as the destination point
            point_type_def target_center;
            bg::centroid(poly, target_center);

            Point targetPoint(target_center.x(), target_center.y());

            std::cout << "CurrentPoint: (" << currPosition.x << ", " << currPosition.y << ")" << std::endl;
            std::cout << "TargetPoint: (" << targetPoint.x << ", " << targetPoint.y << ")" << std::endl;

            auto outPair = getVoronoiGraph(toAvoid, borders, currPosition, targetPoint);
            auto graph = outPair.first;
            auto vertices = outPair.second;

            auto firstIdx = findVertexIndex(vertices, currPosition.x, currPosition.y);
            auto first = vertex(firstIdx, graph);
            auto goalIdx = findVertexIndex(vertices, targetPoint.x, targetPoint.y);
            auto goal = vertex(goalIdx, graph);

            std::vector<int> d(num_vertices(graph));
            std::vector <vertex_descriptor> p(num_vertices(graph));

            drawGraphImage(graph, vertices, currPosition, targetPoint, toAvoid);

            // use Dijkstra to compute a distance vector from the start point
            boost::dijkstra_shortest_paths(graph, first, boost::predecessor_map(&p[0]).distance_map(&d[0]));

            std::cout << "dijkstra distance: " << d[goal] << std::endl;

            if(d[goal] == INT_MAX) {
                std::cerr << "[ERR] Unreachable destination!" << std::endl;
                return false;
            }

            std::vector <vertex_descriptor> tmp_path;
            vertex_descriptor current = goal;

            // using the predecessor array compute the shortest path
            while (current != first) {
                tmp_path.push_back(current);
                current = p[current];
            }
            tmp_path.push_back(first);

            // save the points composing the final path
            std::vector<vertex_descriptor>::reverse_iterator it = tmp_path.rbegin();
            vertex_descriptor v = *it;
            it++;
            while (it != tmp_path.rend()) {
                vertex_descriptor v1 = *it;
                pointPath.push_back(Point(vertices[v].x() / OBJECTS_SCALE_FACTOR,
                                          vertices[v].y() / OBJECTS_SCALE_FACTOR));

                ++it;
                v = v1;
            }

            currPosition = targetPoint;
            prevTarget = targetPoly;
        }
        return true;
    }
}
