#ifndef DUBINS_H
#define DUBINS_H

#include <vector>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <sstream>

#include "utils.hpp"

namespace student {
    struct RobotPosition {
        float x, y, theta;

        RobotPosition(float x, float y, float theta) :
                x(x), y(y), theta(theta) {}

        RobotPosition(Point p, float theta) :
                RobotPosition(p.x, p.y, theta) {}

        RobotPosition() :
                RobotPosition(0, 0, 0) {}
    };

    struct DubinsParams {
        float theta_0, theta_f, k_max, lambda;
    };

    struct DubinsResult {
        float s1, s2, s3;

        DubinsResult() {
            s1 = -1;
            s2 = -1;
            s3 = -1;
        }

        float getSum() {
            return s1 + s2 + s3;
        }

        void scaleFromStandard(const DubinsParams &params) {
            s1 *= params.lambda;
            s2 *= params.lambda;
            s3 *= params.lambda;
        }
    };

    class DubinsCase {
    private:
        int k1Sign, k2Sign, k3Sign;
    public:
        float k1, k2, k3;

        DubinsCase() {}

        DubinsCase(int _k1Sign, int _k2Sign, int _k3Sign) {
            k1Sign = _k1Sign;
            k2Sign = _k2Sign;
            k3Sign = _k3Sign;
        }

        DubinsResult solve(const DubinsParams &params, float kmax) {
            k1 = k1Sign * kmax;
            k2 = k2Sign * kmax;
            k3 = k3Sign * kmax;

            return compute(params);
        }

        virtual DubinsResult compute(const DubinsParams &params) = 0;
    };

    class Dubins {
    private:
        DubinsParams getDubinsParams(const RobotPosition &start, const RobotPosition &end, double kmax);

        std::vector <Pose> getArcPoses(const RobotPosition &start, const DubinsResult &res, const DubinsCase &dubCase);

        RobotPosition getCircLine(const Pose &pos);

        std::vector <Pose> sampleDubinsArc(const Pose &pose);

    public:
        std::vector <Pose> solveDubinsProblem(const RobotPosition &start, const RobotPosition &end, double kmax, float &minLength);
    };

    float sinc(float x);

    float mod2pi(float x);

    class LSL : public DubinsCase {
    public:
        LSL() : DubinsCase(1, 0, 1) {}

        DubinsResult compute(const DubinsParams &params) override;
    };

    class RSR : public DubinsCase {
    public:
        RSR() : DubinsCase(-1, 0, -1) {}

        DubinsResult compute(const DubinsParams &params) override;
    };

    class LSR : public DubinsCase {
    public:
        LSR() : DubinsCase(1, 0, -1) {}

        DubinsResult compute(const DubinsParams &params) override;
    };

    class RSL : public DubinsCase {
    public:
        RSL() : DubinsCase(-1, 0, 1) {}

        DubinsResult compute(const DubinsParams &params) override;
    };

    class RLR : public DubinsCase {
    public:
        RLR() : DubinsCase(-1, 1, -1) {}

        DubinsResult compute(const DubinsParams &params) override;
    };

    class LRL : public DubinsCase {
    public:
        LRL() : DubinsCase(1, -1, 1) {}

        DubinsResult compute(const DubinsParams &params) override;
    };
}


#endif
