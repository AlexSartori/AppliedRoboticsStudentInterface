#ifndef PLANNING_H
#define PLANNING_H

#include "utils.hpp"
#include "voronoi.hpp"

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/types.hpp"

#include "boost/polygon/voronoi.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>

#include "../../libs/clipper/clipper.hpp"

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>

#define OBJECTS_SCALE_FACTOR 500.0


inline bool operator==(const Point &a, const Point &b)
{
    return a.x == b.x && a.y == b.y;
}

namespace student {
    namespace bg = boost::geometry;
    namespace ob = ompl::base;
    namespace og = ompl::geometric;

    typedef bg::model::d2::point_xy<double> point_type_def;
    typedef bg::model::polygon <point_type_def> polygon_type_def;

    Polygon scaleUpPolygon(const Polygon &poly);

    bool elaborateVoronoi(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                          const std::vector <std::pair<int, Polygon>> &victim_list,
                          const Polygon &gate, const Point &robot, std::vector<Point> &pointPath);

    bool rrt_star_planning(const Polygon &borders, const std::vector <Polygon> &toAvoid,
                           const Point &currPosition, const Point &targetPosition,
                           std::vector<Point> &pointPath);
} // namespace student

#endif
