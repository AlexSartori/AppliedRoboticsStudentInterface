#include "dubins_multipoint.hpp"

using namespace std;

namespace student {

    /*!
     * Compute the table for the dynamic programming solution of the Dubins multipoint problem
     * @param path The vector of points that compose the path
     * @return The resulting table
     */
    vector <vector<vector < float>>> DubinsMultipoint::computeDynTable(const vector <Point> &path) {
        int k = angles.size();
        vector < vector < vector < float>>> dynTable(path.size() - 1);
        for (int i = 0; i < dynTable.size(); i++)
            dynTable[i].resize(k, vector<float>(k));

        Dubins d;

        for (int p = 0; p < path.size() - 1; p++) {
            Point pi = path[p];
            Point pj = path[p + 1];
            for (int i = 0; i < k; i++) {
                for (int j = 0; j < k; j++) {
                    RobotPosition start(pi, angles[i]);
                    RobotPosition end(pj, angles[j]);
                    float minLength;
                    std::vector <Pose> poses = d.solveDubinsProblem(start, end, this->kmax, minLength);

                    dynTable[p][i][j] = minLength;
                }
            }
        }

        return dynTable;
    }

    /*!
     * Recursive function that search the best combination of Dubins curves
     * @param index Index specifying the current point of the path
     * @param jAngleIndex Index specifying the angle of the previous point
     * @param dynTable The table for the dynamic programming
     * @param tmpValues Table storing the distances of the best solutions
     * @param tmpIndexes Table storing the index of the angles of the best solutions
     * @return
     */
    float DubinsMultipoint::idpLfunction(int index, int jAngleIndex, const vector<vector<vector<float>>> &dynTable, vector<vector<float>> &tmpValues, vector<vector<int>> &tmpIndexes) {
        if(index == tmpIndexes.size())
            return 0.;

        if(tmpIndexes[index][jAngleIndex] == -1) {
            float minVal = FLT_MAX;
            int minIdx = -1;
            for(int i = 0; i < angles.size();i++) {
                float currValue = dynTable[index][jAngleIndex][i];
                currValue += idpLfunction(index + 1, i, dynTable, tmpValues, tmpIndexes);
                if(currValue<minVal) {
                    minVal = currValue;
                    minIdx = i;
                }
            }

            tmpValues[index][jAngleIndex] = minVal;
            tmpIndexes[index][jAngleIndex] = minIdx;
        }

        return tmpValues[index][jAngleIndex];
    }

    /*!
     * Using dynamic programming it calculates the best combination of Poses to follow the given path
     * @param path The path to follow
     * @param resultPath The resulting vector of Poses
     */
    void DubinsMultipoint::getShortestPath(const vector <Point> &path, Path &resultPath) {
        auto dynTable = computeDynTable(path);

        vector<vector<float>> tmpValues(path.size()-1);
        vector<vector<int>> tmpIndexes(path.size()-1);
        for (int i = 0; i < tmpValues.size(); i++){
            tmpValues[i].resize(angles.size(), 0);
            tmpIndexes[i].resize(angles.size(), -1);
        }

        float length = idpLfunction(0, 0, dynTable, tmpValues, tmpIndexes);

        Dubins d;
        float s = 0;

        int angleI = 0;
        RobotPosition pi(path[0], angles[0]);
        for(int i = 0; i < path.size()-1; i++) {
            int angleJ = tmpIndexes[i][angleI];
            float thetaJ = angles[angleJ];
            RobotPosition pj(path[i+1], thetaJ);
            float minLength;
            std::vector <Pose> poses = d.solveDubinsProblem(pi, pj, this->kmax, minLength);

            for(auto &pose : poses){
                s += pose.s;
                pose.s = s;
                resultPath.points.push_back(pose);
            }

            angleI = angleJ;
            pi = pj;
        }
    }
}