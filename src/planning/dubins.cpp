#include "dubins.hpp"

using namespace std;

namespace student {

    /*!
     * Calculate the parameters of a standard Dubins problem
     * @param start The starting position of the robot
     * @param end The final position of the robot
     * @param kmax The maximum curvature of the robot
     * @return The Dubins parameters
     */
    DubinsParams Dubins::getDubinsParams(const RobotPosition &start, const RobotPosition &end, double kmax) {
        DubinsParams par;

        float dx = end.x - start.x, dy = end.y - start.y;
        float phi = atan2(dy, dx);
        par.lambda = sqrt(pow(dy, 2) + pow(dx, 2)) / 2.;
        par.theta_0 = start.theta - phi;
        par.theta_f = end.theta - phi;
        par.k_max = par.lambda * kmax;

        return par;
    }

    /*!
     * Solve a Dubins problem
     * @param start The starting position of the robot
     * @param end The final position of the robot
     * @param kmax The maximum curvature of the robot
     * @param minLength The resulting minimum length of the path
     * @return A path composed by a vector of Poses
     */
    vector <Pose> Dubins::solveDubinsProblem(const RobotPosition &start, const RobotPosition &end, double kmax, float &minLength) {
        DubinsParams par = getDubinsParams(start, end, kmax);

        DubinsCase* cases[] = {
                new LSL(),
                new RSR(),
                new LSR(),
                new RSL(),
                new RLR(),
                new LRL()
        };

        DubinsResult bestResult;
        DubinsCase* bestCase;
        minLength = FLT_MAX;
        for (auto dubCase : cases) {
            auto res = dubCase->solve(par, kmax);
            float s = res.getSum();
            if (s >= 0 && s < minLength) {
                minLength = s;
                bestResult = res;
                bestCase = dubCase;
            }
        }

        bestResult.scaleFromStandard(par);

        vector <Pose> dubinsPoses = getArcPoses(start, bestResult, *bestCase);
        vector <Pose> poses;
        for (auto pose : dubinsPoses) {
            vector <Pose> tmp = sampleDubinsArc(pose);
            for(auto p : tmp){
                poses.push_back(p);
            }
        }

        return poses;
    }

    /*!
     * Sample starting from a Pose to obtain a more complete path
     * @param pose The initial pose
     * @return A vector of Poses composing the path
     */
    vector <Pose> Dubins::sampleDubinsArc(const Pose &pose) {
        vector <Pose> poses;

        float delta = 0.01;
        for (float s = 0; s < pose.s; s += delta) {
            Pose p(s, pose.x, pose.y, pose.theta, pose.kappa);
            RobotPosition pFinal = getCircLine(p);
            poses.emplace_back(delta, pFinal.x, pFinal.y, pFinal.theta, pose.kappa);
        }

        return poses;
    }

    /*!
     * Calculate the three Poses of a Dubins movement
     * @param start The starting position of the robot
     * @param res The result of a Dubins movement
     * @param dubCase The Dubins case that solved the problem
     * @return A list with the three Poses that form a Dubins movement
     */
    vector <Pose> Dubins::getArcPoses(const RobotPosition &start, const DubinsResult &res, const DubinsCase &dubCase) {
        vector <Pose> poses;

        Pose a1(res.s1, start.x, start.y, start.theta, dubCase.k1);
        RobotPosition a1Final = getCircLine(a1);

        Pose a2(res.s2, a1Final.x, a1Final.y, a1Final.theta, dubCase.k2);
        RobotPosition a2Final = getCircLine(a2);

        Pose a3(res.s3, a2Final.x, a2Final.y, a2Final.theta, dubCase.k3);

        poses.push_back(a1);
        poses.push_back(a2);
        poses.push_back(a3);

        return poses;
    }

    /*!
     * Calculate the final position of the robot after the given circular movement
     * @param pos The circular movement that the robot has to do
     * @return The final robot position
     */
    RobotPosition Dubins::getCircLine(const Pose &pos) {
        RobotPosition p;
        p.x = pos.x + pos.s * sinc(pos.kappa * pos.s / 2.) * cos(pos.theta + pos.kappa * pos.s / 2.);
        p.y = pos.y + pos.s * sinc(pos.kappa * pos.s / 2.) * sin(pos.theta + pos.kappa * pos.s / 2.);
        p.theta = mod2pi(pos.theta + pos.kappa * pos.s);

        return p;
    }

    /*!
     * Numerically stable implementation of the sinc function
     * @param x The parameter of the sinc
     * @return The result of the sinc operation
     */
    float sinc(float x) {
        if (abs(x) < 0.002)
            return 1. - pow(x, 2) / 6. * (1. - pow(x, 2) / 20.);
        else
            return sin(x) / x;
    }

    /*!
     * Normalize the given angle in the range [0, 2pi)
     * @param x The angle to be normalized
     * @return The normalized angle
     */
    float mod2pi(float x) {
        while (x < 0)
            x += 2 * M_PI;
        while (x >= 2 * M_PI)
            x -= 2 * M_PI;
        return x;
    }

    /*!
     * Solve the Dubins problem with a LSL movement
     * @param p The Dubins parameters
     * @return The solution of the Dubins problem
     */
    DubinsResult LSL::compute(const DubinsParams &p) {
        DubinsResult res;

        float invK = 1. / p.k_max;
        float C = cos(p.theta_f) - cos(p.theta_0);
        float S = 2 * p.k_max + sin(p.theta_0) - sin(p.theta_f);
        float temp1 = atan2(C, S);
        float sc_s1 = invK * mod2pi(temp1 - p.theta_0);
        float temp2 = 2. + 4. * pow(p.k_max, 2) - 2. * cos(p.theta_0 - p.theta_f) + 4. * p.k_max * (sin(p.theta_0) - sin(p.theta_f));
        if (temp2 < 0) {
            return res;
        }
        res.s1 = sc_s1;
        res.s2 = invK * sqrt(temp2);
        res.s3 = invK * mod2pi(p.theta_f - temp1);
        return res;
    }

    /*!
     * Solve the Dubins problem with a RSR movement
     * @param p The Dubins parameters
     * @return The solution of the Dubins problem
     */
    DubinsResult RSR::compute(const DubinsParams &p) {
        DubinsResult res;
        float invK = 1. / p.k_max;
        float C = cos(p.theta_0) - cos(p.theta_f);
        float S = 2. * p.k_max - sin(p.theta_0) + sin(p.theta_f);
        float temp1 = atan2(C, S);
        float sc_s1 = invK * mod2pi(p.theta_0 - temp1);
        float temp2 = 2 + 4 * pow(p.k_max, 2) - 2 * cos(p.theta_0 - p.theta_f) - 4 * p.k_max * (sin(p.theta_0) - sin(p.theta_f));
        if (temp2 < 0) {
            return res;
        }
        res.s1 = sc_s1;
        res.s2 = invK * sqrt(temp2);
        res.s3 = invK * mod2pi(temp1 - p.theta_f);
        return res;
    }

    /*!
     * Solve the Dubins problem with a LSR movement
     * @param p The Dubins parameters
     * @return The solution of the Dubins problem
     */
    DubinsResult LSR::compute(const DubinsParams &p) {
        DubinsResult res;
        float invK = 1. / p.k_max;
        float C = cos(p.theta_0) + cos(p.theta_f);
        float S = 2. * p.k_max + sin(p.theta_0) + sin(p.theta_f);
        float temp1 = atan2(-C, S);
        float temp3 = 4 * pow(p.k_max, 2) - 2 + 2 * cos(p.theta_0 - p.theta_f) + 4 * p.k_max * (sin(p.theta_0) + sin(p.theta_f));
        if (temp3 < 0){
            return res;
        }
        res.s2 = invK * sqrt(temp3);
        float temp2 = -atan2(-2, res.s2 * p.k_max);
        res.s1 = invK * mod2pi(temp1 + temp2 - p.theta_0);
        res.s3 = invK * mod2pi(temp1 + temp2 - p.theta_f);
        return res;
    }

    /*!
     * Solve the Dubins problem with a RSL movement
     * @param p The Dubins parameters
     * @return The solution of the Dubins problem
     */
    DubinsResult RSL::compute(const DubinsParams &p) {
        DubinsResult res;
        float invK = 1. / p.k_max;
        float C = cos(p.theta_0) + cos(p.theta_f);
        float S = 2. * p.k_max - sin(p.theta_0) - sin(p.theta_f);
        float temp1 = atan2(C, S);
        float temp3 = 4 * pow(p.k_max, 2) - 2 + 2 * cos(p.theta_0 - p.theta_f) - 4 * p.k_max * (sin(p.theta_0) + sin(p.theta_f));
        if (temp3 < 0){
            return res;
        }
        res.s2 = invK * sqrt(temp3);
        float temp2 = atan2(2, res.s2 * p.k_max);
        res.s1 = invK * mod2pi(p.theta_0 - temp1 + temp2);
        res.s3 = invK * mod2pi(p.theta_f - temp1 + temp2);
        return res;
    }

    /*!
     * Solve the Dubins problem with a RLR movement
     * @param p The Dubins parameters
     * @return The solution of the Dubins problem
     */
    DubinsResult RLR::compute(const DubinsParams &p) {
        DubinsResult res;
        float invK = 1. / p.k_max;
        float C = cos(p.theta_0) - cos(p.theta_f);
        float S = 2. * p.k_max - sin(p.theta_0) + sin(p.theta_f);
        float temp1 = atan2(C, S);
        float temp2 = 0.125 * (6 - 4 * pow(p.k_max, 2) + 2 * cos(p.theta_0 - p.theta_f) + 4 * p.k_max * (sin(p.theta_0) - sin(p.theta_f)));
        if (abs(temp2) > 1){
            return res;
        }
        res.s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        res.s1 = invK * mod2pi(p.theta_0 - temp1 + 0.5 * res.s2 * p.k_max);
        res.s3 = invK * mod2pi(p.theta_0 - p.theta_f + p.k_max * (res.s2 - res.s1));
        return res;
    }

    /*!
     * Solve the Dubins problem with a LRL movement
     * @param p The Dubins parameters
     * @return The solution of the Dubins problem
     */
    DubinsResult LRL::compute(const DubinsParams &p) {
        DubinsResult res;
        float invK = 1. / p.k_max;
        float C = cos(p.theta_f) - cos(p.theta_0);
        float S = 2 * p.k_max + sin(p.theta_0) - sin(p.theta_f);
        float temp1 = atan2(C, S);
        float temp2 = 0.125 * (6 - 4 * pow(p.k_max, 2) + 2 * cos(p.theta_0 - p.theta_f) - 4 * p.k_max * (sin(p.theta_0) - sin(p.theta_f)));
        if (abs(temp2) > 1) {
            return res;
        }
        res.s2 = invK * mod2pi(2 * M_PI - acos(temp2));
        res.s1 = invK * mod2pi(temp1 - p.theta_0 + 0.5 * res.s2 * p.k_max);
        res.s3 = invK * mod2pi(p.theta_f - p.theta_0 + p.k_max * (res.s2 - res.s1));
        return res;
    }
}
