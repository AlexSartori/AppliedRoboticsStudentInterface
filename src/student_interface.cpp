#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <stdexcept>
#include <sstream>
#include <ctime>
#include <atomic>
#include <unistd.h>
#include <sstream>
#include <cmath>
#include <experimental/filesystem>

#include "find_objects.cpp"


namespace student {
  void loadImage(cv::Mat& img_out, const std::string& config_folder){  
    throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
  }

  void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    static std::string folderPath;
    folderPath = config_folder + "/camera_images";
    bool exist = std::experimental::filesystem::exists(folderPath);
    if(!exist){
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

  static std::vector<cv::Point2f> extrinsic_calib_points;
  static cv::Mat extrinsic_calib_bg;
  static std::atomic<bool> extrinsic_calib_done;

  void mouseCallback(int event, int x, int y, int, void* p) {
    if (event != cv::EVENT_LBUTTONDOWN || extrinsic_calib_done.load()) return;
    
    extrinsic_calib_points.emplace_back(x, y);
    cv::circle(extrinsic_calib_bg, cv::Point(x,y), 20, cv::Scalar(0,0,255), -1);
    cv::imshow("Pick 4 points", extrinsic_calib_bg);

    if (extrinsic_calib_points.size() == 4) {
      usleep(500*1000);
      extrinsic_calib_done.store(true);
    }
  }

  std::vector<cv::Point2f> pick4Points(const cv::Mat& img) {
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

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    std::cout << "Entering extrinsicCalib - student implementation" << std::endl;

    // Let the user pick 4 points lying on the same plane  
    std::vector<cv::Point2f> image_points;
    image_points = pick4Points(img_in);

    // Calculate the perspective transformation to apply
    cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!ok)
      std::cerr << "FAILED SOLVE_PNP" << std::endl;
    return ok;
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder) {
    cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder) {

    cv::Mat image_points;
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);
    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const std::string& config_folder) {
    cv::warpPerspective(img_in, img_out, transf, img_in.size());   
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    getVictims(img_in, victim_list);
    getObstacles(img_in, obstacle_list);
    bool res = getGate(img_in, gate);

    // cv::waitKey(1);
    return res;
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){    
    // Converto RGB to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Filter with blue mask
    cv::Mat robot_img;
    cv::inRange(hsv_img, cv::Scalar(80, 66, 33), cv::Scalar(121, 255, 255), robot_img);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(robot_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "Detected " << contours.size() << " contour(s)" << std::endl;

    // Check it is a triangle
    std::vector<cv::Point> approx_curve;
    for(int i = 0; i < contours.size(); i++) {
      cv::approxPolyDP(contours[i], approx_curve, 20, true);
      if (approx_curve.size() == 3) {
        for (const auto& pt: approx_curve)
          triangle.emplace_back(pt.x, pt.y);
        break;
      }
    }
    
    if (triangle.size() != 3) {
        std::cerr << "Error while detecting robot:polygon has " << triangle.size() << " vertices." << std::endl;
        return false;
    }
    
    // Draw robot
    cv::Mat contours_img = img_in.clone();
    cv::polylines(contours_img, approx_curve, true, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
    cv::imshow("Robot", contours_img);
    while (cv::waitKey(100) != 'q') ;
    cv::destroyWindow("Robot");
    
    // Calculate baricenter
    x = 0; y = 0;
    for (Point p : triangle) {
        x += p.x/3;
        y += p.y/3;
    }
    x /= scale;
    y /= scale;
    
    // Find P3 to calculate theta
    double max_dist = 0;
    Point p3;
    for (Point p : triangle) {
        double d = std::pow(p.y-y, 2) + std::pow(p.x-x, 2);
        d = std::sqrt(d);
        
        if (d >= max_dist) {
            max_dist = d;
            p3 = p;
        }
    }
    
    theta = std::atan2(y-p3.y, x-p3.x);
    
    std::cout << "Found robot at x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
    return true;
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );     
  }


}
