#ifndef __DETECTOR_H
#define __DETECTOR_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define DICT_4X4_50 0
#define DICT_5X5_50 4
#define DICT_6X6_50 8

using namespace std;
using namespace cv;
using namespace aruco;

namespace detector{

class Detector
{
public:
    // Interface function
    bool init(string Config_File);
    void detect(Mat& img, vector<int>& ids, vector<Point2f>& centers);
    void getcenter(vector<Point2f>& centers);
    void drawmarker(Mat& img, CvScalar color = Scalar(0, 0, 255), int thickness = 10);
    bool readDetectorParameters(string filename, Ptr<DetectorParameters>& params);
private:
    //

private:
    // Object
    Ptr<DetectorParameters> detectorParams;
    Ptr<Dictionary> dictionary;
    vector<vector<Point2f> > corners;
};


}

#endif // __DETECTOR_H
