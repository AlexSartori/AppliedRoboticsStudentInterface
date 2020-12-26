#include "planning.hpp"

namespace student {
    bool elaborateVoronoi(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                          const std::vector <std::pair<int, Polygon>> &victim_list,
                          const Polygon &gate, const Point &robot, std::vector<Point> &pointPath) {
        std::vector <Polygon> allObjects(obstacle_list);
        for (auto victim : victim_list)
            allObjects.push_back(victim.second);
        allObjects.push_back(gate);

        Point end(400, 450);

        auto outPair = getVoronoiGraph(allObjects, borders, robot, end);
        auto graph = outPair.first;
        auto vertices = outPair.second;

        auto firstIdx = findVertexIndex(vertices, robot.x, robot.y);
        auto first = vertex(firstIdx, graph);
        auto goalIdx = findVertexIndex(vertices, end.x, end.y);
        auto goal = vertex(goalIdx, graph);

        std::vector<int> d(num_vertices(graph));
        std::vector <vertex_descriptor> p(num_vertices(graph));

        boost::dijkstra_shortest_paths(graph, first, boost::predecessor_map(&p[0]).distance_map(&d[0]));

        std::vector <vertex_descriptor> tmp_path;
        vertex_descriptor current = goal;

        while (current != first) {
            tmp_path.push_back(current);
            current = p[current];
        }
        tmp_path.push_back(first);

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

        // now the final path is showed
        std::vector<vertex_descriptor>::reverse_iterator it = tmp_path.rbegin();
        vertex_descriptor v = *it;
        it++;
        while (it != tmp_path.rend()) {
            vertex_descriptor v1 = *it;
            cv::Point cvstart = cv::Point(vertices[v].x(), vertices[v].y());
            cv::Point cvend = cv::Point(vertices[v1].x(), vertices[v1].y());
            cv::line(tmp, cvstart, cvend, cv::Scalar(0, 255, 0), thickness, lineType);

            pointPath.push_back(Point(vertices[v].x()/OBJECTS_SCALE_FACTOR, vertices[v].y()/OBJECTS_SCALE_FACTOR));

            ++it;
            v = v1;
        }

        cv::circle(tmp, cv::Point(robot.x, robot.y), 5, cv::Scalar(255, 0, 0), CV_FILLED, 8, 0);
        cv::circle(tmp, cv::Point(end.x, end.y), 5, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

        /*
        cv::namedWindow("Path Voronoi", cv::WINDOW_NORMAL);
        cv::imshow("Path Voronoi", tmp);
        cv::waitKey(0);*/

        return true;
    }
} // namespace student
