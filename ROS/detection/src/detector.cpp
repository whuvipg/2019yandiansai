#include "detector.h"


namespace detector{

bool Detector::init(string Config_File)
{
    // Aruco parameters
    detectorParams = DetectorParameters::create();
    if(!readDetectorParameters(Config_File, detectorParams))
    {
        cout << "failed to read configure file" << endl;
        return false;
    }

    // Initial detecter
    int dictionaryId = DICT_5X5_50;
    dictionary = getPredefinedDictionary(PREDEFINED_DICTIONARY_NAME(dictionaryId));

    return true;
}

void Detector::detect(Mat& img, vector<int>& ids, vector<Point2f>& centers)
{
    vector<vector<Point2f> > rejected;
    detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
    getcenter(centers);
    drawDetectedMarkers(img, corners, ids, Scalar(0, 0, 255));
}

void Detector::getcenter(vector<Point2f>& centers)
{
    centers.clear();
    for(int i = 0;i < corners.size();i++)
    {
        vector<Point2f> corner = corners[i];
        Point2f center(0.0, 0.0);
        for(int j = 0;j < corner.size();j++)
        {
            center.x += corner[j].x;
            center.y += corner[j].y;
        }        
        center.x /= corner.size();
        center.y /= corner.size();
        centers.push_back(center);
    }
}
void Detector::drawmarker(Mat& img, CvScalar color, int thickness)
{
    for(int i = 0;i < corners.size();i++)
    {
        vector<Point2f> corner = corners[i];
        line(img, corner[0], corner[1], color, thickness);
        line(img, corner[1], corner[2], color, thickness);
        line(img, corner[2], corner[3], color, thickness);
        line(img, corner[3], corner[0], color, thickness);
    }
}

bool Detector::readDetectorParameters(string filename, Ptr<DetectorParameters>& params)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        return false;
    }
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;

    return true;
}


};




