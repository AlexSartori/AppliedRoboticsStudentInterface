#ifndef DUBINS_MULTIPOINT_H
#define DUBINS_MULTIPOINT_H

#include <vector>
#include <cfloat>

#include "utils.hpp"
#include "dubins.hpp"

namespace student {
    class DubinsMultipoint {
    private:
        std::vector<float> angles;
        float kmax;

        std::vector <std::vector<std::vector < float>>> computeDynTable(const std::vector <Point> &path);

        float idpLfunction(int index, int jAngleIndex, const std::vector<std::vector<std::vector<float>>> &dynTable,
                                            std::vector<std::vector<float>> &tmpValues,
                                            std::vector<std::vector<int>> &tmpIndexes);
    public:
        DubinsMultipoint(int k, float startTheta, float kmax) {
            this->kmax = kmax;

            float delta = 2 * M_PI / k;
            for (int i = 0; i < k; i++)
                angles.push_back(startTheta + delta * i);
        }

        void getShortestPath(const std::vector <Point> &path, Path &resultPath);
    };
}


#endif //DUBINS_MULTIPOINT_H
