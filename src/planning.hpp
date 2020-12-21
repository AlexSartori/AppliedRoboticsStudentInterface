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

#include <stdlib.h>
#include <iostream>
#include <algorithm>

namespace student {
    bool elaborateVoronoi(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                          const std::vector <std::pair<int, Polygon>> &victim_list,
                          const Polygon &gate, const Point &robot, Path &path);
} // namespace student

#endif
