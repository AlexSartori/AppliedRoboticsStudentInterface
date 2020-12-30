#include "find_objects.hpp"


namespace student {
    std::pair<int, int> getDigitFromRoi(cv::Mat &roi) {
        // resize the image to be bigger than the template one
        cv::resize(roi, roi, cv::Size(200, 200));

        // Load digits template images
        std::vector <cv::Mat> templROIs;

        const char *env_root = std::getenv("AR_ROOT");
        std::string baseFolder(env_root);

        for (int i = 0; i <= 9; ++i) {
            templROIs.emplace_back(cv::imread(baseFolder + "/number_templates/" + std::to_string(i) + ".png"));
            if (templROIs[i].empty()) {
                throw std::runtime_error("Can't load templates of the numbers!");
            }
            if (templROIs[i].type() != 0)
                cv::cvtColor(templROIs[i], templROIs[i], CV_BGR2GRAY);
        }

        double maxScore = -1;
        int maxIdx = -1;
        for (int i = 0; i < templROIs.size(); i++) {
            cv::Mat result;
            cv::matchTemplate(roi, templROIs[i], result, cv::TM_CCOEFF);
            double score;
            cv::minMaxLoc(result, nullptr, &score);
            if (score > maxScore) {
                maxScore = score;
                maxIdx = i;
            }
        }

        return {maxIdx, maxScore};
    }

    int extrapolateVictimNumber(cv::Mat &roi) {
        // filter the image
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4), cv::Point(-1, -1));
        cv::dilate(roi, roi, kernel);
        cv::erode(roi, roi, kernel);

        cv::Point2f center(roi.cols / 2., roi.rows / 2.);

        std::vector<int> results_conf;

        for (int i = 0; i < 10; i++) {
            results_conf.push_back(0);
        }
        cv::flip(roi, roi, 1); // flip horizontally

        int minArea = INT_MAX;
        int minIndex = -1;
        bool toRotate = false;

        // rotate the ROI to find the position where the rectangle that covers it has minimum area
        for (int degree = 0; degree < 180; degree += 5) {
            cv::Mat tmp;
            cv::Mat r = cv::getRotationMatrix2D(center, degree, 1.0);
            cv::warpAffine(roi, tmp, r, roi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

            cv::Rect rectangle;
            int area;

            std::vector <std::vector<cv::Point>> contours;
            cv::findContours(tmp, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_L1);
            // find the contour covering 1/3 of the ROI (to remove wrong contours) and it's not the entire ROI image
            for (auto &contour: contours) {
                rectangle = cv::boundingRect(contour);
                area = rectangle.width * rectangle.height;
                if (area >= 1. / 3. * roi.cols * roi.rows && area < roi.cols * roi.rows) {
                    break;
                }
            }

            if (area < minArea) {
                minArea = area;
                minIndex = degree;
                toRotate = (rectangle.width > rectangle.height);
            }
        }

        // take the best rotated ROI
        cv::Mat r = cv::getRotationMatrix2D(center, minIndex, 1.0);
        cv::warpAffine(roi, roi, r, roi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        if (toRotate) { // to have the image vertically
            r = cv::getRotationMatrix2D(center, 90, 1.0);
            cv::warpAffine(roi, roi, r, roi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
        }

        for (int degree = 0; degree < 360; degree += 180) { // rotate the image
            for (int i = -10; i <= 10; i++) {
                cv::Mat tmp;
                r = cv::getRotationMatrix2D(center, degree + i, 1.0);
                cv::warpAffine(roi, tmp, r, roi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                               cv::Scalar(255, 255, 255));

                std::pair<int, int> res = getDigitFromRoi(tmp);

                if (res.first != 0) {
                    if (results_conf[res.first] < res.second)
                        results_conf[res.first] = res.second;
                }
            }
        }

        int max = std::max_element(results_conf.begin(), results_conf.end()) - results_conf.begin();
        std::cout << "recognised " << max << std::endl;
        return max;
    }

    void getVictims(const cv::Mat &img_in, std::vector <std::pair<int, Polygon>> &victim_list, const double scale) {
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

        cv::Mat victims_img;
        cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), victims_img);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8), cv::Point(-1, -1));
        cv::erode(victims_img, victims_img, kernel);
        cv::dilate(victims_img, victims_img, kernel);

        std::vector <std::vector<cv::Point>> contours, approx_contours;
        cv::findContours(victims_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (auto &contour: contours) {
            std::vector <cv::Point> approx_curve;
            cv::approxPolyDP(contour, approx_curve, 7, true);
            if (approx_curve.size() >= 6) {
                approx_contours.push_back(approx_curve);
                Polygon victim;
                for (const auto &pt: approx_curve) {
                    victim.emplace_back(pt.x / scale, pt.y / scale);
                }

                cv::Rect rectangle = cv::boundingRect(contour);
                cv::Point2f center;
                float radius;
                // get the circle that contains the contour
                cv::minEnclosingCircle(approx_curve, center, radius);
                cv::Scalar color = cv::Scalar(255, 255, 255);
                int diagonal = (int) std::sqrt(std::pow(rectangle.width, 2) + std::pow(rectangle.height, 2));

                // increase the radius of the circle to fill with black the parts outside the victim's circle
                for (int i = -3; (int) radius + i <= diagonal / 2; i++)
                    cv::circle(victims_img, center, (int) radius + i, color, 2);

                cv::Mat roi = victims_img(rectangle);

                int id = extrapolateVictimNumber(roi);
                victim_list.push_back({
                                              id,
                                              victim
                                      });
            }
        }

        cv::Mat contours_img = img_in.clone();
        cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

        for (auto &victim: victim_list) {
            // print in the image the ID of the victims
            cv::Point position = cv::Point(victim.second[0].x, victim.second[0].y);
            cv::putText(contours_img, std::to_string(victim.first),
                        position, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0), 2);
        }
        //cv::imshow("Victims", contours_img);
    }

    void getObstacles(const cv::Mat &img_in, std::vector <Polygon> &obstacle_list, const double scale) {
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

        cv::Mat obst_img, obst_img_1, obst_img_2;
        cv::inRange(hsv_img, cv::Scalar(163, 40, 40), cv::Scalar(180, 255, 255), obst_img_1);
        cv::inRange(hsv_img, cv::Scalar(0, 40, 40), cv::Scalar(35, 255, 255), obst_img_2);
        add(obst_img_1, obst_img_2, obst_img);

        cv::Mat erosion_img;
        cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8), cv::Point(-1, -1));
        cv::erode(obst_img, erosion_img, erosion_element);

        cv::Mat dilation_img;
        cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10), cv::Point(-1, -1));
        cv::dilate(erosion_img, dilation_img, dilation_element);

        std::vector <std::vector<cv::Point>> contours, approx_contours;
        cv::findContours(dilation_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            std::vector <cv::Point> approx_curve;
            cv::approxPolyDP(contours[i], approx_curve, 7, true);
            approx_contours.push_back(approx_curve);

            Polygon obstacle;
            for (const auto &pt: approx_curve) {
                obstacle.emplace_back(pt.x / scale, pt.y / scale);
            }
            obstacle_list.push_back(obstacle);
        }

        cv::Mat contours_img = img_in.clone();
        cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

        //cv::imshow("Obstacles", contours_img);
    }

    bool getGate(const cv::Mat &img_in, Polygon &gate, const double scale) {
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

        cv::Mat gate_img;
        cv::inRange(hsv_img, cv::Scalar(45, 50, 0), cv::Scalar(90, 255, 255), gate_img);

        cv::Mat erosion_img;
        cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8), cv::Point(-1, -1));
        cv::erode(gate_img, erosion_img, erosion_element);

        cv::Mat dilation_img;
        cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10), cv::Point(-1, -1));
        cv::dilate(erosion_img, dilation_img, dilation_element);

        std::vector <std::vector<cv::Point>> contours, approx_contours;
        cv::findContours(dilation_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool found = false;
        for (int i = 0; i < contours.size() && !found; i++) {
            double area = cv::contourArea(contours[i]);
            std::vector <cv::Point> approx_curve;
            cv::approxPolyDP(contours[i], approx_curve, 8, true);
            if (approx_curve.size() == 4 && area >= 500) {
                approx_contours.push_back(approx_curve);
                for (const auto &pt: approx_curve) {
                    gate.emplace_back(pt.x / scale, pt.y / scale);
                }
                found = true;
            }
        }

        cv::Mat contours_img = img_in.clone();
        cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

        //cv::imshow("Gate", contours_img);

        return found;
    }
}
