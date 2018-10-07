// kalman_from_opencvexamples.c

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

#include <eigen3/Eigen/Dense>
#include "opencv2/core/eigen.hpp"


#include "../src/kalman/kalman_filter.h"

#include<iostream>

// function prototypes ////////////////////////////////////////////////////////////////////////////
void mouseMoveCallback(int event, int x, int y, int flags, void* userData);
void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color);

// global variables ///////////////////////////////////////////////////////////////////////////////
const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);

cv::Point ptActualMousePosition(0, 0);

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    const int state_dim = 4;
    const int meas_dim  = 2;
    const int ctl_dim   = 0;

    Eigen::Matrix<float, state_dim, state_dim> transition_matrix;
    transition_matrix << 1, 0, 1, 0,
                         0, 1, 0, 1,
                         0, 0, 1, 0,
                         0, 0, 0, 1;

    Eigen::Matrix<float, meas_dim, state_dim> measurement_matrix;
    measurement_matrix << 1, 0, 0, 0,
                          0, 1, 0, 0;
    
    cpppc::Kalman_Filter<state_dim, meas_dim, ctl_dim, float> kafi(transition_matrix, measurement_matrix);

    kafi.set_process_noise(Eigen::Matrix<float, state_dim, state_dim>::Identity() * 0.0001);
    kafi.set_measurement_noise(Eigen::Matrix<float, meas_dim, meas_dim>::Identity() * 10);
    kafi.set_process_cov(Eigen::Matrix<float, state_dim, state_dim>::Identity() * 0.1);

    cv::Mat imgBlank(700, 900, CV_8UC3, cv::Scalar::all(0));            // declare a blank image for moving the mouse over

    std::vector<cv::Point> predictedMousePositions;                 // declare 3 vectors for predicted, actual, and corrected positions
    std::vector<cv::Point> actualMousePositions;
    std::vector<cv::Point> correctedMousePositions;

    cv::namedWindow("imgBlank");                                // declare window
    cv::setMouseCallback("imgBlank", mouseMoveCallback);        // 

    while (true) {

        kafi.predict();

        cv::Mat matPredicted;
        cv::eigen2cv( kafi.pre_state, matPredicted );
        std::cout << "Kafi predicted pos: "
                << "( " << kafi.pre_state(0,0)
                << ", " << kafi.pre_state(1,0)
                << ", " << kafi.pre_state(2,0)
                << ", " << kafi.pre_state(3,0)
                << " )" << '\n';

        cv::Point ptPredicted((int)matPredicted.at<float>(0), (int)matPredicted.at<float>(1));

        cv::Mat matActualMousePosition(2, 1, CV_32F, cv::Scalar::all(0));

        matActualMousePosition.at<float>(0, 0) = (float)ptActualMousePosition.x;
        matActualMousePosition.at<float>(1, 0) = (float)ptActualMousePosition.y;

        // Convert to eigen
        Eigen::Matrix<float, meas_dim, 1> actualMousePosition;
        cv::cv2eigen( matActualMousePosition, actualMousePosition);

        cv::Mat matCorrected;
        kafi.update( actualMousePosition );   // update() updates the predicted state from the measurement

        std::cout << "Kafi updated pos: "
                  << "( " << kafi.post_state(0,0)
                  << ", " << kafi.post_state(1,0)
                  << ", " << kafi.post_state(2,0)
                  << ", " << kafi.post_state(3,0)
                  << " )" << '\n';

        cv::eigen2cv( kafi.post_state, matCorrected );

        cv::Point ptCorrected((int)matCorrected.at<float>(0), (int)matCorrected.at<float>(1));

        predictedMousePositions.push_back(ptPredicted);
        actualMousePositions.push_back(ptActualMousePosition);
        correctedMousePositions.push_back(ptCorrected);

        // predicted, actual, and corrected are all now calculated, time to draw stuff

        drawCross(imgBlank, ptPredicted, SCALAR_BLUE);                      // draw a cross at the most recent predicted, actual, and corrected positions
        drawCross(imgBlank, ptActualMousePosition, SCALAR_WHITE);
        drawCross(imgBlank, ptCorrected, SCALAR_GREEN);
        
        for (int i = 0; i < predictedMousePositions.size() - 1; i++) {                  // draw each predicted point in blue
            cv::line(imgBlank, predictedMousePositions[i], predictedMousePositions[i + 1], SCALAR_BLUE, 1);
        }

        for (int i = 0; i < actualMousePositions.size() - 1; i++) {                     // draw each actual point in white
            cv::line(imgBlank, actualMousePositions[i], actualMousePositions[i + 1], SCALAR_WHITE, 1);
        }

        for (int i = 0; i < correctedMousePositions.size() - 1; i++) {                  // draw each corrected point in green
            cv::line(imgBlank, correctedMousePositions[i], correctedMousePositions[i + 1], SCALAR_GREEN, 1);
        }

        cv::imshow("imgBlank", imgBlank);         // show the image
        
        cv::waitKey(10);                    // pause for a moment to get operating system to redraw the imgBlank

        imgBlank = cv::Scalar::all(0);         // blank the imgBlank for next time around
    }

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void mouseMoveCallback(int event, int x, int y, int flags, void* userData) {
    if (event == CV_EVENT_MOUSEMOVE) {

        ptActualMousePosition.x = x;
        ptActualMousePosition.y = y;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color) {
    cv::line(img, cv::Point(center.x - 5, center.y - 5), cv::Point(center.x + 5, center.y + 5), color, 2);
    cv::line(img, cv::Point(center.x + 5, center.y - 5), cv::Point(center.x - 5, center.y + 5), color, 2);

}
