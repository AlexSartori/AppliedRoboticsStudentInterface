#include "planning.hpp"

namespace student {
    bool victimSortFn (const std::pair<int, Polygon>& struct1, const std::pair<int, Polygon>& struct2) {
        return (struct1.first < struct2.first);
    }

    bool elaborateVoronoi(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                          const std::vector <std::pair<int, Polygon>> &victim_list,
                          const Polygon &gate, const Point &robot, std::vector<Point> &pointPath) {

        Point currPosition(robot);
        Polygon prevTarget;

        std::vector<Polygon> targets;
        std::vector <std::pair<int, Polygon>> sortedVictims(victim_list);

        std::sort (sortedVictims.begin(), sortedVictims.end(), victimSortFn);
        for(auto &v : sortedVictims)
            targets.push_back(v.second);
        //TODO: fix adding gate
        //targets.push_back(gate);

        std::vector <Polygon> allObjects(obstacle_list);
        for (auto victim : victim_list)
            allObjects.push_back(victim.second);
        //allObjects.push_back(gate);

        for(auto &targetPoly : targets) {
            std::vector <Polygon> toAvoid(allObjects);
            toAvoid.erase(std::remove(toAvoid.begin(), toAvoid.end(), targetPoly));
            if (prevTarget.size() > 0)
                toAvoid.erase(std::remove(toAvoid.begin(), toAvoid.end(), prevTarget));

            polygon_type_def poly;
            point_type_def firstPoint;

            for(auto &v : targetPoly){
                point_type_def tmpP(v.x, v.y);
                bg::append(poly.outer(), tmpP);
            }
            point_type_def tmpP(targetPoly[0].x, targetPoly[0].y);
            bg::append(poly.outer(), tmpP);

            point_type_def my_center(0,0);
            bg::centroid(poly, my_center);

            Point targetPoint(my_center.x(), my_center.y());

            std::cout << "CurrentPoint: ("<<currPosition.x << ", "<<currPosition.y<<")" <<std::endl;
            std::cout << "TargetPoint: ("<<targetPoint.x << ", "<<targetPoint.y<<")" <<std::endl;

            auto outPair = getVoronoiGraph(toAvoid, borders, currPosition, targetPoint);
            auto graph = outPair.first;
            auto vertices = outPair.second;

            auto firstIdx = findVertexIndex(vertices, currPosition.x, currPosition.y);
            auto first = vertex(firstIdx, graph);
            auto goalIdx = findVertexIndex(vertices, targetPoint.x, targetPoint.y);
            auto goal = vertex(goalIdx, graph);

            std::vector<int> d(num_vertices(graph));
            std::vector <vertex_descriptor> p(num_vertices(graph));

            boost::dijkstra_shortest_paths(graph, first, boost::predecessor_map(&p[0]).distance_map(&d[0]));

            std::cout << "dijkstra distance: " << d[goal] << std::endl;

            if(d[goal] == INT_MAX) {
                std::cerr << "[ERR] Unreachable destination!" << std::endl;
                return false;
            }

            std::vector <vertex_descriptor> tmp_path;
            vertex_descriptor current = goal;


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
            cv::circle(tmp, cv::Point(currPosition.x, currPosition.y), 5, cv::Scalar(255, 0, 0), CV_FILLED, 8, 0);
            cv::circle(tmp, cv::Point(targetPoint.x, targetPoint.y), 5, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

            for(int i = 1; i < borders.size(); i++){
                cv::Point cvstart = cv::Point(borders[i-1].x, borders[i-1].y);
                cv::Point cvend = cv::Point(borders[i].x, borders[i].y);
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 0), thickness, lineType);
            }

            auto pathImg = "/root/workspace/Graph.png";
            //cv::imwrite(pathImg, tmp);
            //std::cout << "img saved at "<<pathImg << std::endl;

            while (current != first) {
                tmp_path.push_back(current);
                current = p[current];
            }
            tmp_path.push_back(first);

            // now the final path is showed
            std::vector<vertex_descriptor>::reverse_iterator it = tmp_path.rbegin();
            vertex_descriptor v = *it;
            it++;
            while (it != tmp_path.rend()) {
                vertex_descriptor v1 = *it;
                /*cv::Point cvstart = cv::Point(vertices[v].x(), vertices[v].y());
                cv::Point cvend = cv::Point(vertices[v1].x(), vertices[v1].y());
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 255, 0), thickness, lineType);*/

                pointPath.push_back(
                        Point(vertices[v].x() / OBJECTS_SCALE_FACTOR, vertices[v].y() / OBJECTS_SCALE_FACTOR));

                ++it;
                v = v1;
            }

            /*
            cv::namedWindow("Path Voronoi", cv::WINDOW_NORMAL);
            cv::imshow("Path Voronoi", tmp);
            cv::waitKey(0);*/

            currPosition = targetPoint;
            prevTarget = targetPoly;
        }
        return true;
    }
} // namespace student
