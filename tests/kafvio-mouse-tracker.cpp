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
const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar SCALAR_BLUE = cv::Scalar(255.0, 0.0, 0.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

cv::Point ptActualMousePosition(0, 0);

///////////////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

    const int state_dim = 4;
    const int meas_dim  = 2;
    const int ctl_dim   = 0;
    //cv::KalmanFilter kalmanFilter(4, 2, 0);                             // instantiate Kalman Filter
    //Kalman_Filter<state_dim, meas_dim, ctl_dim> kafi;

    Eigen::Matrix<float, state_dim, state_dim> transition_matrix;
    transition_matrix << 1, 0, 1, 0,
                         0, 1, 0, 1,
                         0, 0, 1, 0,
                         0, 0, 0, 1;
    /*float fltTransitionMatrixValues[4][4] = { { 1, 0, 1, 0 },           // declare an array of floats to feed into Kalman Filter Transition Matrix, also known as State Transition Model
                                              { 0, 1, 0, 1 },
                                              { 0, 0, 1, 0 },
                                              { 0, 0, 0, 1 } };
    */
    //kalmanFilter.transitionMatrix = cv::Mat(4, 4, CV_32F, fltTransitionMatrixValues);       // set Transition Matrix
    //kafi.set_transition_mtx( transition_matrix );

    /*float fltMeasurementMatrixValues[2][4] = { { 1, 0, 0, 0 },          // declare an array of floats to feed into Kalman Filter Measurement Matrix, also known as Measurement Model
                                               { 0, 1, 0, 0 } };
    */
    Eigen::Matrix<float, meas_dim, state_dim> measurement_matrix;
    measurement_matrix << 1, 0, 0, 0,
                          0, 1, 0, 0;
    //kalmanFilter.measurementMatrix = cv::Mat(2, 4, CV_32F, fltMeasurementMatrixValues);     // set Measurement Matrix
    //kafi.set_meas_mtx( measurement_matrix );
    
    cpppc::Kalman_Filter<state_dim, meas_dim, ctl_dim, float> kafi(transition_matrix, measurement_matrix);

    //cv::setIdentity(kalmanFilter.processNoiseCov, cv::Scalar::all(0.0001));           // default is 1, for smoothing try 0.0001
    //cv::setIdentity(kalmanFilter.measurementNoiseCov, cv::Scalar::all(10));         // default is 1, for smoothing try 10
    //cv::setIdentity(kalmanFilter.errorCovPost, cv::Scalar::all(0.1));               // default is 0, for smoothing try 0.1
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

        //cv::Mat matPredicted = kalmanFilter.predict();

        cv::Point ptPredicted((int)matPredicted.at<float>(0), (int)matPredicted.at<float>(1));

        cv::Mat matActualMousePosition(2, 1, CV_32F, cv::Scalar::all(0));

        matActualMousePosition.at<float>(0, 0) = (float)ptActualMousePosition.x;
        matActualMousePosition.at<float>(1, 0) = (float)ptActualMousePosition.y;

        // Convert to eigen
        Eigen::Matrix<float, meas_dim, 1> actualMousePosition;
        cv::cv2eigen( matActualMousePosition, actualMousePosition);

        cv::Mat matCorrected;
        kafi.update( actualMousePosition );

        std::cout << "Kafi updated pos: "
                  << "( " << kafi.pre_state(0,0)
                  << ", " << kafi.pre_state(1,0)
                  << " )" << '\n';

        cv::eigen2cv( kafi.pre_state, matCorrected );

        //cv::Mat matCorrected = kalmanFilter.correct(matActualMousePosition);        // function correct() updates the predicted state from the measurement

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
        //std::cout << "mouse move at: " << x << ", " << y << "\n";

        ptActualMousePosition.x = x;
        ptActualMousePosition.y = y;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawCross(cv::Mat &img, cv::Point center, cv::Scalar color) {
    cv::line(img, cv::Point(center.x - 5, center.y - 5), cv::Point(center.x + 5, center.y + 5), color, 2);
    cv::line(img, cv::Point(center.x + 5, center.y - 5), cv::Point(center.x - 5, center.y + 5), color, 2);

}
