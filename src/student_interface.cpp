#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <stdexcept>
#include <sstream>
#include <ctime>
#include <atomic>
#include <unistd.h>
#include <cmath>
#include <experimental/filesystem>

#include "find_objects.hpp"
#include "planning/planning.hpp"
#include "planning/dubins_multipoint.hpp"
#include "config.hpp"


namespace student {
    void loadImage(cv::Mat &img_out, const std::string &config_folder) {
        std::cout << "Load image - Student implementation" << std::endl;
        static bool initialized = false;
        static std::vector <cv::String> img_list; // list of images to load
        static size_t idx = 0;  // idx of the current img
        static size_t function_call_counter = 0;  // idx of the current img
        const static size_t freeze_img_n_step = 30; // hold the current image for n iteration
        static cv::Mat current_img; // store the image for a period, avoid to load it from file every time
        std::string path = config_folder + "/camera_images";

        if (!initialized) {
            const bool recursive = false;
            // Load the list of jpg images contained in the config_folder/img_to_load/
            cv::glob(path + "/*.jpg", img_list, recursive);

            if (img_list.size() > 0) {
                initialized = true;
                idx = 0;
                current_img = cv::imread(img_list[idx]);
                function_call_counter = 0;
            } else {
                initialized = false;
            }
        }

        if (!initialized) {
            throw std::logic_error("Load Image can not find any jpg image in: " + path);
            return;
        }

        img_out = current_img;
        function_call_counter++;

        // If the function is called more than N times load increment image idx
        if (function_call_counter > freeze_img_n_step) {
            function_call_counter = 0;
            idx = (idx + 1) % img_list.size();
            current_img = cv::imread(img_list[idx]);
        }
    }

    void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder) {
        static std::string folderPath;
        folderPath = config_folder + "/camera_images";
        bool exist = std::experimental::filesystem::exists(folderPath);
        if (!exist) {
            std::experimental::filesystem::create_directories(folderPath);
        }

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream dateTime;
        dateTime << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");

        static std::string imgName;
        imgName = folderPath + "/image" + dateTime.str() + ".jpg";
        cv::imwrite(imgName, img_in);
        std::cout << "Saved image " << imgName << " w/ topic: " << topic << std::endl;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////
    ////    Extrinsic Calibration
    /////////////////////////////////////////////////////////////////////////////////////////////

    static std::vector <cv::Point2f> extrinsic_calib_points;
    static cv::Mat extrinsic_calib_bg;
    static std::atomic<bool> extrinsic_calib_done;

    void mouseCallback(int event, int x, int y, int, void *p) {
        if (event != cv::EVENT_LBUTTONDOWN || extrinsic_calib_done.load()) return;

        extrinsic_calib_points.emplace_back(x, y);
        cv::circle(extrinsic_calib_bg, cv::Point(x, y), 20, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Pick 4 points", extrinsic_calib_bg);

        if (extrinsic_calib_points.size() == 4) {
            usleep(500 * 1000);
            extrinsic_calib_done.store(true);
        }
    }

    std::vector <cv::Point2f> pick4Points(const cv::Mat &img) {
        extrinsic_calib_points.clear();
        extrinsic_calib_done.store(false);

        std::string win_name = "Pick 4 points";
        extrinsic_calib_bg = img.clone();
        cv::imshow(win_name, extrinsic_calib_bg);
        cv::namedWindow(win_name);

        cv::setMouseCallback(win_name, &mouseCallback, nullptr);
        while (!extrinsic_calib_done.load()) {
            cv::waitKey(500);
        }

        cv::destroyWindow(win_name);
        return extrinsic_calib_points;
    }

    bool extrinsicCalib(const cv::Mat &img_in, std::vector <cv::Point3f> object_points, const cv::Mat &camera_matrix,
                        cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder) {
        // Let the user pick 4 points lying on the same plane
        std::vector <cv::Point2f> image_points;
        image_points = pick4Points(img_in);

        // Calculate the perspective transformation to apply
        cv::Mat dist_coeffs;
        dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
        bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

        if (!ok)
            std::cerr << "[ERR] Failed cv::solvePnP" << std::endl;
            
        return ok;
    }

    void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs,
                        const std::string &config_folder) {
        cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
    }

    void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                            const cv::Mat &tvec, const std::vector <cv::Point3f> &object_points_plane,
                            const std::vector <cv::Point2f> &dest_image_points_plane,
                            cv::Mat &plane_transf, const std::string &config_folder) {

        cv::Mat image_points;
        cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);
        plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
    }


    void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder) {
        cv::warpPerspective(img_in, img_out, transf, img_in.size());
    }

    bool processMap(const cv::Mat &img_in, const double scale, std::vector <Polygon> &obstacle_list,
                    std::vector <std::pair<int, Polygon>> &victim_list, Polygon &gate,
                    const std::string &config_folder) {
                    
        int vision_start_time = time(NULL);
        getVictims(img_in, victim_list, scale);
        getObstacles(img_in, obstacle_list, scale);
        bool res = getGate(img_in, gate, scale);
        int vision_end_time = time(NULL);
        std::cout << "[TIME] Computer vision phase completed in " << vision_end_time - vision_start_time << "s" << std::endl;

        std::cout << "READY TO RUN!" << std::endl;
        std::cout << "Recognised digits: ";
        for(auto & v : victim_list)
            std::cout << v.first << " ";
        std::cout << std::endl;

        return res;
    }

    bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta,
                   const std::string &config_folder) {
        // Converto RGB to HSV
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

        // Filter with blue mask
        cv::Mat robot_img;
        cv::inRange(hsv_img, cv::Scalar(80, 66, 33), cv::Scalar(121, 255, 255), robot_img);

        // Find contours
        std::vector <std::vector<cv::Point>> contours;
        cv::findContours(robot_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Check it is a triangle
        std::vector <cv::Point> approx_curve;
        for (int i = 0; i < contours.size(); i++) {
            cv::approxPolyDP(contours[i], approx_curve, 20, true);
            if (approx_curve.size() == 3) {
                for (const auto &pt: approx_curve)
                    triangle.emplace_back(pt.x, pt.y);
                break;
            }
        }

        if (triangle.size() != 3) {
            std::cerr << "[ERR] Error while detecting robot: polygon has " << triangle.size() << " vertices." << std::endl;
            return false;
        }

        // Calculate baricenter
        x = y = 0;
        for (Point p : triangle) {
            x += p.x / 3;
            y += p.y / 3;
        }

        // Find P3 to calculate theta
        double max_dist = 0;
        Point p3;
        for (Point p : triangle) {
            double d = std::pow(p.y - y, 2) + std::pow(p.x - x, 2);
            d = std::sqrt(d);

            if (d >= max_dist) {
                max_dist = d;
                p3 = p;
            }
        }

        theta = std::atan2(y - p3.y, x - p3.x);

        if(readBooleanConfig("debug_robot")) {
            // Draw robot
            cv::Mat contours_img = img_in.clone();
            cv::polylines(contours_img, approx_curve, true, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            cv::circle(contours_img, cv::Point(p3.x, p3.y), 5, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
            cv::circle(contours_img, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
            cv::imshow("Robot", contours_img);
            while (cv::waitKey(100) != 'q') ;
            cv::destroyWindow("Robot");

            std::cout << "Found robot at x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
        }

        x /= scale;
        y /= scale;
        
        return true;
    }

    /*!
     * Save the image of the arena with highlighted the path that the robot wants to take
     * @param borders the borders of the arena
     * @param obstacle_list The list of the obstacles
     * @param victim_list The list of the victims
     * @param gate The final gate
     * @param path The path for the robot (the vector of poses)
     * @param pointPath The points of the graph
     */
    void savePlannedImage(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                          const std::vector <std::pair<int, Polygon>> &victim_list,
                          const Polygon &gate, const Path &path, const std::vector<Point> &pointPath) {
        cv::Mat tmp(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        int thickness = 2;
        int lineType = cv::LINE_8;

        // draw the borders
        for(int i = 0; i < borders.size(); i++) {
            cv::Point cvstart = cv::Point(borders[i].x, borders[i].y);
            cv::Point cvend = cv::Point(borders[(i+1) % borders.size()].x, borders[(i+1) % borders.size()].y);
            cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 0), thickness, lineType);
        }

        // draw obstacles
        for(auto &obs : obstacle_list) {
            for(int i = 0; i < obs.size(); i++) {
                cv::Point cvstart = cv::Point(obs[i].x, obs[i].y);
                cv::Point cvend = cv::Point(obs[(i+1) % obs.size()].x, obs[(i+1) % obs.size()].y);
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 0, 255), thickness, lineType);
            }
        }
        // draw victims
        for(auto &vic : victim_list) {
            for(int i = 0; i < vic.second.size(); i++) {
                cv::Point cvstart = cv::Point(vic.second[i].x, vic.second[i].y);
                cv::Point cvend = cv::Point(vic.second[(i+1) % vic.second.size()].x, vic.second[(i+1) % vic.second.size()].y);
                cv::line(tmp, cvstart, cvend, cv::Scalar(0, 255, 0), thickness, lineType);
            }
        }
        // draw gate
        for(int i = 0; i < gate.size(); i++) {
            cv::Point cvstart = cv::Point(gate[i].x, gate[i].y);
            cv::Point cvend = cv::Point(gate[(i+1) % gate.size()].x, gate[(i+1) % gate.size()].y);
            cv::line(tmp, cvstart, cvend, cv::Scalar(0, 255, 0), thickness, lineType);
        }

        // draw final path points
        for (auto &p: path.points) {
            cv::circle(tmp, cv::Point(p.x * OBJECTS_SCALE_FACTOR, p.y*OBJECTS_SCALE_FACTOR), 5, cv::Scalar(255, 0, 0), CV_FILLED, 2, 0);
        }
        // draw graph path points
        for (auto &p: pointPath) {
            cv::circle(tmp, cv::Point(p.x * OBJECTS_SCALE_FACTOR, p.y*OBJECTS_SCALE_FACTOR), 5, cv::Scalar(0, 255, 0), CV_FILLED, 2, 0);
        }

        const char *env_root = std::getenv("AR_ROOT");
        std::string baseFolder(env_root);
        auto pathImg = baseFolder + "/plan.png";
        cv::imwrite(pathImg, tmp);

        std::cout << "img saved at "<<pathImg << std::endl;
    }

    bool planPath(const Polygon &borders, const std::vector <Polygon> &obstacle_list,
                  const std::vector <std::pair<int, Polygon>> &victim_list,
                  const Polygon &gate, const float x, const float y, const float theta,
                  Path &path,
                  const std::string &config_folder) {

        Polygon newBorders = scaleUpPolygon(borders);
        Polygon newGate = scaleUpPolygon(gate);

        std::vector <Polygon> newObstacles;
        for(auto &obst : obstacle_list)
            newObstacles.push_back(scaleUpPolygon(obst));

        std::vector <std::pair<int, Polygon>> newVictims;
        for(auto &vict : victim_list)
            newVictims.push_back({
                vict.first,
                scaleUpPolygon(vict.second)
            });

        Point robot(x * OBJECTS_SCALE_FACTOR, y * OBJECTS_SCALE_FACTOR);

        std::vector<Point> pointPath;
        int rrt_start_time = time(NULL);
        bool result = elaboratePath(newBorders, newObstacles, newVictims, newGate, robot, pointPath);
        int rrt_end_time = time(NULL);
        std::cout << "[TIME] RRT planning completed in " << rrt_end_time - rrt_start_time << "s" << std::endl;
        
        if (result) {
            std::cout << "Found a path with " << pointPath.size() << " points" << std::endl;

            float kmax = readFloatConfig("maximum_curvature");
            int k = readIntConfig("n_dubins_angles");
            
            int dubins_start_time = time(NULL);
            DubinsMultipoint* dp = new DubinsMultipoint(k, theta, kmax);
            dp->getShortestPath(pointPath, path);
            int dubins_end_time = time(NULL);
            std::cout << "[TIME] Dubins curves computation completed in " << dubins_end_time - dubins_start_time << "s" << std::endl;
            
            std::cout << "[TIME] Total planning time: " << dubins_end_time - rrt_start_time << "s" << std::endl;
        } else {
            std::cerr << "[ERR] Could not find a valid path" << std::endl;
        }

        if(readBooleanConfig("debug_path"))
            savePlannedImage(newBorders, newObstacles, newVictims, newGate, path, pointPath);

        return result;
    }
}
