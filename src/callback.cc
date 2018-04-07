#include <iostream>
#include <sys/time.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <utility>

#include "callback.h"
#include "sensor_params.h"


const std::vector<BalloonInfo> processImage(const cv::Mat& img) {
    /* Sensor params in: sensorParams */
    /* GPS solution in: gpsSolution */

    /* Define what red and blue are */
    typedef struct {
        Color color;
        std::vector< std::pair<cv::Scalar, cv::Scalar> > bounds; 
    } Filter;

    const cv::Scalar redBoundLower1(155, 100, 100);
    const cv::Scalar redBoundUpper1(180, 255, 255);
    const cv::Scalar redBoundLower2(0, 100, 100);
    const cv::Scalar redBoundUpper2(25, 255, 255);
    const std::vector< std::pair<cv::Scalar, cv::Scalar> > redBounds = {
        std::make_pair(redBoundLower1, redBoundUpper1),
        std::make_pair(redBoundLower2, redBoundUpper2)
        };
    const Filter redFilter = {red, redBounds};

    const cv::Scalar blueBoundLower1(75, 100, 100);
    const cv::Scalar blueBoundUpper1(125, 255, 255);
    const std::vector< std::pair<cv::Scalar, cv::Scalar> > blueBounds = {
        std::make_pair(blueBoundLower1, blueBoundUpper1)
        };
    const Filter blueFilter = {blue, blueBounds};

    const std::vector<Filter> filters = {redFilter, blueFilter};


    // Crop the image
	cv::Mat cp = img.clone();
	cv::Rect ROI(0, 0, sensor_params.image_width-1, sensor_params.image_height-1);
	cv::Mat frame = cp(ROI);

    // Image properties
    const uint16_t rows = frame.rows;
    const uint16_t cols = frame.cols;

    // Return vector
    std::vector<BalloonInfo> infoVec;

    // Blur image and convert it to the HSV color space
    cv::GaussianBlur(frame, frame, cv::Size(11, 11), 0);
    cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

    for(Filter filter: filters) { 
        const Color color = filter.color;

        // Filter out non-specified colors
        cv::Mat binary = cv::Mat(rows, cols, CV_8U, double(0));
        cv::Mat buffer;;
        for(auto& bounds: filter.bounds) {
            cv::inRange(frame, bounds.first, bounds.second, buffer);
            cv::bitwise_or(buffer, binary, binary);
        }

        cv::erode(binary, binary, 0, cv::Point(-1, -1), 2);
        cv::dilate(binary, binary, 0, cv::Point(-1, -1), 2);

        // Find contours in image
        std::vector< std::vector<cv::Point> > contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Contours must have greater than 5 points. Required for ellipse in next
        // section.
        contours.erase(std::remove_if(contours.begin(), contours.end(), [](std::vector<cv::Point> v){return v.size() < 5;}), contours.end());

        // Loop through the contours 
        // Reduce the set by applying the following criteria:
        // 1) Bounding rectangle width and height must be above a threshhold
        // 2) Ratio between expected and calculated area must be under a threshhold
        // 3) Ratio between expected and calculated perimeter must be under a threshhold
        // 4) Contour is a certain distance away from the edge of the image
        // 5) Ratio between hull area and contour area must be above a threshhold
        const uint16_t sizeThreshhold = 50;
        const double areaThreshhold = 0.05;
        const double perimeterThreshhold = 0.05;
        const double hullThreshhold = 0.75;
        const uint16_t imageEdgeThreshhold = 5;

        std::vector< std::vector<cv::Point> > candidateContours;

        auto selectCandidateContours = [&](const std::vector<cv::Point> contour) {

            const double minX = std::min_element(contour.begin(), contour.end(), 
                        [](const cv::Point& a, const cv::Point& b) -> bool
                        { return a.x < b.x; })->x;
            const double minY = std::min_element(contour.begin(), contour.end(), 
                        [](const cv::Point& a, const cv::Point& b) -> bool
                        { return a.y < b.y; })->y;
            const double maxX = std::max_element(contour.begin(), contour.end(), 
                        [](const cv::Point& a, const cv::Point& b) -> bool
                        { return a.x < b.x; })->x;
            const double maxY = std::max_element(contour.begin(), contour.end(), 
                        [](const cv::Point& a, const cv::Point& b) -> bool
                        { return a.y < b.y; })->y;


            const cv::RotatedRect boundingRect = cv::fitEllipse(contour);
            const double width  = boundingRect.size.width;
            const double height = boundingRect.size.height;

            const double contourArea = cv::contourArea(contour);

            // Not the most efficient. Work around because of opencv bug?
            cv::Mat mHull;
            cv::convexHull(cv::Mat(contour), mHull, false, true);
            std::vector<cv::Point> hull;
            hull.assign(mHull.begin<cv::Point>(), mHull.end<cv::Point>());
            const double hullArea = cv::contourArea(hull);
            const double hullPerimeter = cv::arcLength(hull, true);

            // Expected parameters based on perfect ellipse
            const double expectedPerimeter = M_PI * (width + height) / 2.0;
            const double expectedArea = M_PI * width * height / 4;

            if(
                minX > imageEdgeThreshhold && 
                minY > imageEdgeThreshhold &&
                maxY < rows - imageEdgeThreshhold &&
                maxX < cols - imageEdgeThreshhold &&
                width > sizeThreshhold &&
                height > sizeThreshhold &&
                contourArea / hullArea > hullThreshhold &&
                std::fabs(expectedArea - hullArea)/expectedArea < areaThreshhold &&
                std::fabs(expectedPerimeter - hullPerimeter)/expectedPerimeter < perimeterThreshhold
            ) {
             candidateContours.push_back(contour);   
            }
        };

        // Apply the selection criteria
        std::for_each(contours.begin(), contours.end(), selectCandidateContours);


        auto circleCandidateContours = [&](std::vector< cv::Point > contour) {
            // Minimum bounding circle
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            // Center of hull
            cv::Moments m = cv::moments(contour);
            cv::Point2f centerHull((m.m10 / m.m00), (m.m01 / m.m00));

            BalloonInfo info;
            info.balloonRadius = radius;
            info.balloonLocation = Eigen::Vector3d(centerHull.x, centerHull.y, 0);
            info.color = color;
            infoVec.push_back(info);

            if(color == red) {
                std::cout << "Found red balloon!" << std::endl;
            } else if(color == blue) {
                std::cout << "Found blue balloon!" << std::endl;
            }

        };

        // Circle all the candidate contours
        std::for_each(candidateContours.begin(), candidateContours.end(), circleCandidateContours);
    }

    return infoVec;

}
