#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"


namespace student {
  void getVictims(const cv::Mat& img_in, std::vector<std::pair<int, Polygon>>& victim_list){
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat victims_img;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 0), cv::Scalar(90, 255, 255), victims_img);

    cv::Mat erosion_img;
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8), cv::Point(-1,-1));
    cv::erode(victims_img, erosion_img, erosion_element); 
    
    cv::Mat dilation_img;
    cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10), cv::Point(-1,-1));
    cv::dilate(erosion_img, dilation_img, dilation_element); 

    std::vector<std::vector<cv::Point>> contours, approx_contours;
    cv::findContours(erosion_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int id = 0;
    for(auto& contour: contours){
      std::vector<cv::Point> approx_curve;
      cv::approxPolyDP(contour, approx_curve, 7, true);
      if (approx_curve.size() >= 6){
        approx_contours.push_back(approx_curve);
        Polygon victim;
        for (const auto& pt: approx_curve) {
          victim.emplace_back(pt.x, pt.y);
        }
        victim_list.push_back({
          id, 
          victim
        });

        //TODO: get the id from the image
        id++;
      }
    }

    cv::Mat contours_img = img_in.clone();
    cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
          
    cv::imshow("Victims", contours_img);
  }

  void getObstacles(const cv::Mat& img_in, std::vector<Polygon>& obstacle_list){
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat obst_img, obst_img_1, obst_img_2;
    cv::inRange(hsv_img, cv::Scalar(163, 40, 40), cv::Scalar(180, 255, 255), obst_img_1);
    cv::inRange(hsv_img, cv::Scalar(0, 40, 40), cv::Scalar(35, 255, 255), obst_img_2);
    add(obst_img_1, obst_img_2, obst_img);

    cv::Mat erosion_img;
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8), cv::Point(-1,-1));
    cv::erode(obst_img, erosion_img, erosion_element); 
    
    cv::Mat dilation_img;
    cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10), cv::Point(-1,-1));
    cv::dilate(erosion_img, dilation_img, dilation_element); 

    std::vector<std::vector<cv::Point>> contours, approx_contours;
    cv::findContours(erosion_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(int i = 0; i < contours.size(); i++){
      std::vector<cv::Point> approx_curve;
      cv::approxPolyDP(contours[i], approx_curve, 7, true);
      approx_contours.push_back(approx_curve);

      Polygon obstacle;
      for (const auto& pt: approx_curve) {
        obstacle.emplace_back(pt.x, pt.y);
      }
      obstacle_list.push_back(obstacle);
    }

    cv::Mat contours_img = img_in.clone();
    cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
                                          
    cv::imshow("Obstacles", contours_img);
  }

  bool getGate(const cv::Mat& img_in, Polygon& gate){
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat gate_img;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 0), cv::Scalar(90, 255, 255), gate_img);

    cv::Mat erosion_img;
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8), cv::Point(-1,-1));
    cv::erode(gate_img, erosion_img, erosion_element); 
    
    cv::Mat dilation_img;
    cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10), cv::Point(-1,-1));
    cv::dilate(erosion_img, dilation_img, dilation_element); 

    std::vector<std::vector<cv::Point>> contours, approx_contours;
    cv::findContours(erosion_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool found = false;
    for(int i = 0; i < contours.size() && !found; i++){
      std::vector<cv::Point> approx_curve;
      cv::approxPolyDP(contours[i], approx_curve, 8, true);
      if (approx_curve.size() == 4){
        approx_contours.push_back(approx_curve);
        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x, pt.y);
        }
        found = true;
      }
    }

    cv::Mat contours_img = img_in.clone();
    cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
          
    cv::imshow("Gate", contours_img);

    return found;
  }
}
