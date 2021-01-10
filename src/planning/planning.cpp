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
     * Convert a std::vector<Point> object to a boost::geometry::polygon.
     * @param input The polygon to be converted
     */
    polygon_type_def convertPolygonToBoost(const Polygon &input) {
        polygon_type_def poly;

        for(auto &v : input){
            point_type_def tmpP(v.x, v.y);
            bg::append(poly.outer(), tmpP);
        }
        
        point_type_def tmpP(input[0].x, input[0].y);
        bg::append(poly.outer(), tmpP);

        return poly;
    }


    /*!
     * Convert a boost::geometry::polygon to a std::vector<Point> object.
     * @param input The polygon to be converted
     */
    Polygon convertBoostToPolygon(const polygon_type_def &input) {
        Polygon poly;

        for(point_type_def p : input.outer()) {
            poly.emplace_back(p.x(), p.y());
        }
        
        return poly;
    }


    /*!
     * Calculate the centroid of a given polygon.
     * @param input The polygon
     */
    Point calculateCentroid(const Polygon &input) {
        polygon_type_def poly = convertPolygonToBoost(input);
        point_type_def poly_center;

        bg::centroid(poly, poly_center);
        return Point(poly_center.x(), poly_center.y());
    }

    
    /*!
     * Expand the given polygons and merge those which overlap.
     * @param polygons The polygons to buffer
     */
    void expandAndMerge(std::vector<Polygon> &polygons) {
        bg::model::multi_polygon<polygon_type_def> boostPolygons, boostResult;
        
        for (Polygon p : polygons) {
            polygon_type_def poly = convertPolygonToBoost(p);
            boostPolygons.push_back(poly);
        }
        
        float offset = readFloatConfig("polygon_offset");
        bg::strategy::buffer::distance_symmetric<int> distanceStrategy(offset * OBJECTS_SCALE_FACTOR);
        bg::strategy::buffer::side_straight sideStrategy;
        bg::strategy::buffer::join_round joinStrategy(20); // Points per circle
        bg::strategy::buffer::end_round endStrategy(20);
        bg::strategy::buffer::point_circle pointStrategy(20);
    
        bg::buffer(
            boostPolygons, boostResult,
            distanceStrategy, sideStrategy, joinStrategy, endStrategy, pointStrategy
        );
        
        polygons.clear();
        for (polygon_type_def p : boostResult) {
            Polygon poly = convertBoostToPolygon(p);
            polygons.push_back(poly);
        }
    }


    /*!
     * Decide the list of target Polygons that the robot has to reach sorted by priority
     * @param borders The borders of the map, to be avoided
     * @param obstacle_list The vector of obstacle polygons to be avoided
     * @param victim_list The vector of victims to be rescued
     * @param gate The polygon in which the path has to end
     * @param currPosition The current position of the robot
     * @return a vector of target Polygons
     */
    std::vector <Polygon> getMissionTargets(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                                            const std::vector <std::pair<int, Polygon>> &victim_list,
                                            const Polygon &gate, const Point &currPosition) {
        std::vector <Polygon> targets;
        int mission = readIntConfig("mission");
        float max_distance = readFloatConfig("miss2_dist_threshold") * OBJECTS_SCALE_FACTOR;

        std::vector <std::pair<int, Polygon>> targetVictims;
        if (mission == 1) {
            // We have to reach all the victims
            targetVictims = victim_list;
        } else if (mission == 2) {
            std::vector <Polygon> obstacles(obstacle_list);
            expandAndMerge(obstacles);

            // Use the centroid of the gate as the destination point
            Point gatePoint = calculateCentroid(gate);

            std::vector <Point> pointPath;
            bool res = rrt_star_planning(borders, obstacles, currPosition, gatePoint, pointPath);

            if (res) {
                // Construct a line composed of the point of the path to the gate
                linestring_type_def boostPath;
                for (auto &p : pointPath)
                    boostPath.push_back(point_type_def(p.x * OBJECTS_SCALE_FACTOR, p.y * OBJECTS_SCALE_FACTOR));

                for (auto &v : victim_list) {
                    polygon_type_def victim = convertPolygonToBoost(v.second);
                    // Check the distance between the path and the victim
                    if (bg::distance(victim, boostPath) <= max_distance) {
                        // Find the index of the nearest point in the path
                        int pathIndex = -1;
                        float minDist = FLT_MAX;
                        for (int i = 0; i < boostPath.size(); i++) {
                            float d = bg::distance(victim, boostPath[i]);
                            if (d < minDist) {
                                minDist = d;
                                pathIndex = i;
                            }
                        }
                        // Insert the victim and the index of the nearest point as the priority
                        targetVictims.push_back({pathIndex, v.second});
                        std::cout <<"v_id: "<<v.first << " poIndex: " << pathIndex << std::endl;
                    }
                }
            }
        } else {
            std::cerr << "[ERR] Wrong mission number!" << std::endl;
            return targets;
        }

        // Insert the victims as targets sorted by priority
        std::sort(targetVictims.begin(), targetVictims.end(), victimSortFn);
        for (auto &v : targetVictims)
            targets.push_back(v.second);
        targets.push_back(gate);

        return targets;
    }
    

    /*!
     * Elaborate a path that touches all victims in order and avoids obstacles.
     * @param borders The borders of the map, to be avoided
     * @param obstacle_list The vector of obstacle polygons to be avoided
     * @param victim_list The vector of victims to be rescued
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

        // Get the targets of this mission
        std::vector<Polygon> targets = getMissionTargets(borders, obstacle_list, victim_list, gate, currPosition);

        // An array of all the possible objects present in the arena
        std::vector <Polygon> allObjects;
        for (Polygon p : obstacle_list)
            allObjects.push_back(p);
        for (auto victim : victim_list)
            allObjects.push_back(victim.second);
        allObjects.push_back(gate);

        for(auto &targetPoly : targets) {
            // Array of objects to avoid
            std::vector <Polygon> toAvoid(allObjects);
            toAvoid.erase(std::remove(toAvoid.begin(), toAvoid.end(), targetPoly));

            expandAndMerge(toAvoid);

            // Use the centroid of the target as the destination point
            Point targetPoint = calculateCentroid(targetPoly);

            std::cout << "CurrentPoint: (" << currPosition.x << ", " << currPosition.y << ")" << std::endl;
            std::cout << "TargetPoint: (" << targetPoint.x << ", " << targetPoint.y << ")" << std::endl;

            bool res = rrt_star_planning(borders, toAvoid, currPosition, targetPoint, pointPath);
            
            if(!res)
                return false;

            currPosition = targetPoint;
            allObjects.erase(std::remove(allObjects.begin(), allObjects.end(), targetPoly));
        }
        return true;
    }


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

            return true;
        }

    };


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
        obj->setCostThreshold(ob::Cost(0.5 * OBJECTS_SCALE_FACTOR));
        pdef->setOptimizationObjective(obj);

        // Construct the optimizing planner using the RRTstar algorithm.
        ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();

        // Try to solve the planning problem within one second
        float rrt_exec_seconds = readFloatConfig("rrt_exec_seconds");
        ob::PlannerStatus solved = optimizingPlanner->solve(rrt_exec_seconds);
        
        if (!solved)
            return false;

        // Get the solution
        ob::PlannerData data(si);
        optimizingPlanner->getPlannerData(data);

        ob::PathLengthOptimizationObjective opt(si);
        data.computeEdgeWeights(opt);

        // Get the graph and extract the shortest path
        ob::PlannerData::Graph::Type& graph = data.toBoostGraph();
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);
        ob::PlannerData::Graph::Vertex startv = boost::vertex(data.getStartIndex(0), graph);
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

        boost::dijkstra_shortest_paths(
                graph,
                startv,
                boost::predecessor_map(prev).
                    distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
                    distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
                    distance_inf(opt.infiniteCost()).
                    distance_zero(opt.identityCost())
        );

        og::PathGeometric path(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
                 prev[pos] != pos;
                 pos = prev[pos]) {
            path.append(vertices[pos]->getState());
        }
        path.reverse();

        for(auto &s : path.getStates()) {
            double x = s->as<ob::RealVectorStateSpace::StateType>()->values[0];
            double y = s->as<ob::RealVectorStateSpace::StateType>()->values[1];
            pointPath.push_back(Point(x / OBJECTS_SCALE_FACTOR, y / OBJECTS_SCALE_FACTOR));
        }

        return true;
    }
}
