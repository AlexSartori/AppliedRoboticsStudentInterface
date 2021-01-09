#include "planning.hpp"

namespace student {


    /*!
     * Comparator function to sort victims based on their number.
     */
    bool victimSortFn (const std::pair<int, Polygon>& struct1, const std::pair<int, Polygon>& struct2) {
        return (struct1.first < struct2.first);
    }


    /*!
     * Scale up the given polygon to improve precision when working with integer coordinates.
     */
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
    

    /*!
     * Expand (offset) a given polygon.
     * @param polygon The polygon to be expanded
     */
    void expandPolygon(Polygon &polygon) {
        ClipperLib::Path p;
        ClipperLib::Paths solution;

        for (Point pt : polygon) {
            p << ClipperLib::IntPoint(pt.x * OBJECTS_SCALE_FACTOR, pt.y * OBJECTS_SCALE_FACTOR);
        }
        
        ClipperLib::ClipperOffset co;
        co.AddPath(p, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(solution, 30 * OBJECTS_SCALE_FACTOR);

        polygon.clear();
        for (ClipperLib::Path sol : solution)
            for (ClipperLib::IntPoint pt : sol)
                polygon.emplace_back(pt.X / OBJECTS_SCALE_FACTOR, pt.Y / OBJECTS_SCALE_FACTOR);
    }


    /*!
     * Merge polygons that overlap.
     * @param polygons A vector of polygons to be merged where they overlap
     */
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


    /*!
     * Convert a std::vector<Point> objects to a boost::geometry::polygon.
     * @param input The polygon to be converted
     */
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
    

    /*!
     * Plot the Voronoi graph overlayed on the map to an image.
     */
    void drawGraphImage(const UndirectedGraph &graph, const std::vector<bp::voronoi_vertex<double>> &vertices,
                        const Point &currPosition, const Point &targetPoint, const std::vector <Polygon> &obstacles) {
        cv::Mat tmp(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        int thickness = 2;
        int lineType = cv::LINE_8;

        // Draw graph edges
        std::pair <edge_iterator, edge_iterator> edgePair;
        for (edgePair = edges(graph); edgePair.first != edgePair.second; ++edgePair.first) {
            auto a = vertices[source(*edgePair.first, graph)];
            auto b = vertices[target(*edgePair.first, graph)];

            cv::Point cvstart = cv::Point(a.x(), a.y());
            cv::Point cvend = cv::Point(b.x(), b.y());
            cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 0), thickness, lineType);
        }

        // Draw obstacles
        for (auto &obst : obstacles){
            for (int i = 1; i <= obst.size(); i++){
                cv::Point cvstart = cv::Point(obst[i % obst.size()].x, obst[i % obst.size()].y);
                cv::Point cvend = cv::Point(obst[i-1].x, obst[i-1].y);
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 255), thickness, lineType);
            }
        }

        // Plot current path segment
        cv::circle(tmp, cv::Point(currPosition.x, currPosition.y), 5, cv::Scalar(255, 0, 0), CV_FILLED, 8, 0);
        cv::circle(tmp, cv::Point(targetPoint.x, targetPoint.y), 5, cv::Scalar(0, 255, 0), CV_FILLED, 8, 0);

        auto pathImg = "/root/workspace/Graph.png";
        cv::imwrite(pathImg, tmp);
        //std::cout << "img saved at "<<pathImg << std::endl;

        //cv::namedWindow("Path Voronoi", cv::WINDOW_NORMAL);
        //cv::imshow("Path Voronoi", tmp);
        //cv::waitKey(0);
    }
    

    /*!
     * Elaborate a path that touches all victims in order and avoids obstacles.
     * @param borders The borders of the map, to be avoided
     * @param obstacle_list The vector of obstacle polygons to be avoided
     * @param vistim_list The vector of victims to be rescued
     * @param gate The polygon in which the path has to end
     * @param robot The current position of the robot
     * @param pointPath A vector of points where the result will be stored
     * @return `true` if a path was found, `false` otherwise
     */
    bool elaboratePath(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                       const std::vector <std::pair<int, Polygon>> &victim_list,
                       const Polygon &gate, const Point &robot, std::vector<Point> &pointPath) {

        Point currPosition(robot);
        Polygon prevTarget;
        pointPath.emplace_back(currPosition.x / OBJECTS_SCALE_FACTOR, currPosition.y / OBJECTS_SCALE_FACTOR);

        // Create a vector of targets sorted by priority
        std::vector<Polygon> targets;
        std::vector<std::pair<int, Polygon>> sortedVictims(victim_list);
        std::sort(sortedVictims.begin(), sortedVictims.end(), victimSortFn);
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

            for (int i = 0; i < toAvoid.size(); i++)
                expandPolygon(toAvoid[i]);
            
            // Merge together overlapping obstacles
            joinOverlappingPolygons(toAvoid);

            // Use the centroid of the target as the destination point
            polygon_type_def poly = convertPolygonToBoost(targetPoly);
            point_type_def target_center;
            bg::centroid(poly, target_center);
            Point targetPoint(target_center.x(), target_center.y());

            std::cout << "CurrentPoint: (" << currPosition.x << ", " << currPosition.y << ")" << std::endl;
            std::cout << "TargetPoint: (" << targetPoint.x << ", " << targetPoint.y << ")" << std::endl;

            bool res = rrt_star_planning(borders, toAvoid, currPosition, targetPoint, pointPath);
            
            if(!res)
                return false;

            currPosition = targetPoint;
            prevTarget = targetPoly;
        }
        return true;
    }


    /*bool voronoi_planning(const Polygon &borders, const std::vector <Polygon> &toAvoid,
                          const Point &currPosition, const Point &targetPosition,
                          std::vector<Point> &pointPath) {
        auto outPair = getVoronoiGraph(toAvoid, borders, currPosition, targetPosition);
        auto graph = outPair.first;
        auto vertices = outPair.second;

        auto firstIdx = findVertexIndex(vertices, currPosition.x, currPosition.y);
        auto first = vertex(firstIdx, graph);
        auto goalIdx = findVertexIndex(vertices, targetPosition.x, targetPosition.y);
        auto goal = vertex(goalIdx, graph);

        std::vector<int> d(num_vertices(graph));
        std::vector <vertex_descriptor> p(num_vertices(graph));

        drawGraphImage(graph, vertices, currPosition, targetPosition, toAvoid);

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
        return true;
    }*/


    /*!
     * Container class for the `isValid` method needed by the RRT* planning
     */
    class ValidityChecker : public ob::StateValidityChecker {
    public:
        std::vector<Polygon> obstacles;
        ValidityChecker(const ob::SpaceInformationPtr& si, std::vector<Polygon> toAvoid) :
            ob::StateValidityChecker(si) {
            obstacles = toAvoid;
        }

        /*!
         * Verify that the given point is outside of any obstacle polygon.
         */
        bool isValid(const ob::State* state) const {
            const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();
            double x = state2D->values[0];
            double y = state2D->values[1];
            
            // Check that the given point doesn't intersect with any obstacle
            point_type_def boostPoint(x, y);
            for (Polygon polygon : obstacles) {
                polygon_type_def boostPoly = convertPolygonToBoost(polygon);
                if (boost::geometry::within(boostPoint, boostPoly))
                    return false;
            }

            //TODO: check that the robot is not touching a border

            return true;
        }

    };


    // Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
    ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                               const ob::GoalState* goal,
                               const ob::OptimizationObjective* obj,
                               const boost::property_map<ob::PlannerData::Graph::Type,
                                       vertex_type_t>::type& plannerDataVertices)
    {
        return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
    }


    /*!
     * Use the RRT* algorithm to find a valid path between the current position and the destination point.
     */
    bool rrt_star_planning(const Polygon &borders, const std::vector <Polygon> &toAvoid,
                           const Point &currPosition, const Point &targetPosition,
                           std::vector<Point> &pointPath) {
        ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

        float maxX = 0, maxY = 0;
        for(auto p : borders) {
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
        }
        ob::RealVectorBounds spaceBounds(2);
        spaceBounds.setLow(0.0);
        spaceBounds.setHigh(0, maxX);
        spaceBounds.setHigh(1, maxY);
        space->as<ob::RealVectorStateSpace>()->setBounds(spaceBounds);

        // Construct a space information instance and provide the implemented ValidityChecker
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si, toAvoid)));
        si->setup();

        ob::ScopedState<> start(space);
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = currPosition.x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = currPosition.y;

        ob::ScopedState<> goal(space);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = targetPosition.x;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = targetPosition.y;

        // Create a problem instance
        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
        pdef->setStartAndGoalStates(start, goal);

        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(1.51));
        pdef->setOptimizationObjective(obj);

        // Construct the optimizing planner using the RRTstar algorithm.
        ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // Try to solve the planning problem within one second
        ob::PlannerStatus solved = optimizingPlanner->solve(1.0);
        if (!solved)
            return false;

        // Get the solution
        ob::PlannerData data(si);
        optimizingPlanner->getPlannerData(data);

        ob::PathLengthOptimizationObjective opt(si);
        data.computeEdgeWeights(opt);

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

        // Now we can apply any Boost.Graph algorithm.  How about A*!
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

        // Run A* search over our planner data
        ob::GoalState goals(si);
        goals.setState(data.getGoalVertex(0).getState());
        ob::PlannerData::Graph::Vertex startv = boost::vertex(data.getStartIndex(0), graph);
        boost::astar_visitor<boost::null_visitor> dummy_visitor;
        boost::astar_search(graph, startv,
                            [&goals, &opt, &vertices](ob::PlannerData::Graph::Vertex v1) { return distanceHeuristic(v1, &goals, &opt, vertices); },
                            boost::predecessor_map(prev).
                                    distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
                                    distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
                                    distance_inf(opt.infiniteCost()).
                                    distance_zero(opt.identityCost()).
                                    visitor(dummy_visitor));

        // Extracting the path
        og::PathGeometric path(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
                 prev[pos] != pos;
                 pos = prev[pos]) {
            path.append(vertices[pos]->getState());
        }
        path.reverse();

        std::cout << "num states: "<< path.getStates().size() << std::endl;

        for(auto &s : path.getStates()) {
            double x = s->as<ob::RealVectorStateSpace::StateType>()->values[0];
            double y = s->as<ob::RealVectorStateSpace::StateType>()->values[1];
            pointPath.push_back(Point(x / OBJECTS_SCALE_FACTOR, y / OBJECTS_SCALE_FACTOR));
        }

        return true;
    }
}
