#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/types.hpp"

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <stdlib.h>


namespace student {
  std::pair<int, int> getDigitFromRoi(const cv::Mat& roi){
    tesseract::TessBaseAPI* ocr = new tesseract::TessBaseAPI();
    ocr->Init(NULL, "eng");
    ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
    ocr->SetVariable("tessedit_char_whitelist","0123456789");

    // the following three instructions must be directly consecutive
    ocr->SetImage(roi.data, roi.cols, roi.rows, roi.channels(), roi.step);
    char* text = ocr->GetUTF8Text();
    int conf = ocr->MeanTextConf();

    int number = atoi(text);
    ocr->End();

    std::pair<int, int> result = {number, conf};
    return result;
  }

  int extrapolateVictimNumber(const cv::Mat& roi) {
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1,-1));
    cv::erode(roi, roi, erosion_element);

    cv::Point2f center(roi.cols/2., roi.rows/2.);

    std::vector<int> results_conf;

    for (int i = 0; i < 10; i++){
      results_conf.push_back(0);
    }

    for (int degree = 0; degree <= 360; degree += 30) {
      cv::Mat r = cv::getRotationMatrix2D(center, degree, 1.0);

      cv::Mat dst, flipped;
      cv::warpAffine(roi, dst, r, roi.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255,255,255));
      cv::flip(dst, flipped, 1); // flip horizontally

      std::pair<int, int> res = getDigitFromRoi(flipped);

      if(res.first != 0) {
        if(results_conf[res.first] < res.second)
          results_conf[res.first] = res.second;
      }
      //cv::waitKey(0);
    }

    int max = std::max_element(results_conf.begin(), results_conf.end()) - results_conf.begin();
    std::cout << "recognised " << max << std::endl;
    //cv::waitKey(0);
    return max;
  }

  void getVictims(const cv::Mat& img_in, std::vector<std::pair<int, Polygon>>& victim_list){
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat victims_img;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), victims_img);

    cv::Mat erosion_img;
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8), cv::Point(-1,-1));
    cv::erode(victims_img, erosion_img, erosion_element); 
    
    cv::Mat dilation_img;
    cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10), cv::Point(-1,-1));
    cv::dilate(erosion_img, dilation_img, dilation_element); 

    std::vector<std::vector<cv::Point>> contours, approx_contours;
    cv::findContours(dilation_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(auto& contour: contours){
      std::vector<cv::Point> approx_curve;
      cv::approxPolyDP(contour, approx_curve, 7, true);
      if (approx_curve.size() >= 6){
        approx_contours.push_back(approx_curve);
        Polygon victim;
        for (const auto& pt: approx_curve) {
          victim.emplace_back(pt.x, pt.y);
        }

        cv::Rect rectangle = cv::boundingRect(contour);
        cv::Point2f center;
        float radius;
        // get the circle that contains the contour
        cv::minEnclosingCircle(approx_curve, center, radius);
        cv::Scalar color = cv::Scalar(255, 255, 255); // white color
        int diagonal = (int)std::sqrt(std::pow(rectangle.width,2) + std::pow(rectangle.height,2));

        // increase the radius of the circle to fill with black the parts outside the victim's circle
        for(int i = -3; (int)radius + i <= diagonal / 2; i++)
          cv::circle(dilation_img, center, (int)radius + i, color, 2);

        cv::Mat roi = dilation_img(rectangle);
        
        int id = extrapolateVictimNumber(roi);
        victim_list.push_back({
          id, 
          victim
        });
      }
    }

    cv::Mat contours_img = img_in.clone();
    cv::drawContours(contours_img, approx_contours, -1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);

    for(auto& victim: victim_list){
      // print in the image the ID of the victims
      cv::Point position = cv::Point(victim.second[0].x, victim.second[0].y);
      cv::putText(contours_img, std::to_string(victim.first),
                  position, cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 0, 0), 2);
    }
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
    cv::findContours(dilation_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
    cv::findContours(dilation_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
