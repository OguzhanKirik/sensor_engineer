
#include <iostream>
#include <algorithm>
#include <numeric>
#include <limits>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
    else
    {
        cv::waitKey(1); // refresh window without blocking
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> candidateMatches;
    std::vector<double> distances;

    for (const auto &match : kptMatches)
    {
        const cv::KeyPoint &prevKpt = kptsPrev.at(match.queryIdx);
        const cv::KeyPoint &currKpt = kptsCurr.at(match.trainIdx);

        if (boundingBox.roi.contains(currKpt.pt))
        {
            candidateMatches.push_back(match);
            distances.push_back(cv::norm(currKpt.pt - prevKpt.pt));
        }
    }

    if (candidateMatches.empty())
    {
        return;
    }

    const double meanDistance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    for (size_t i = 0; i < distances.size(); ++i)
    {
        if (distances[i] < meanDistance)
        {
            boundingBox.kptMatches.push_back(candidateMatches[i]);
            boundingBox.keypoints.push_back(kptsCurr[candidateMatches[i].trainIdx]);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    std::vector<double> distRatios;
    const double minDist = 100.0;

    for (size_t i = 0; i < kptMatches.size(); ++i)
    {
        const cv::KeyPoint &kpOuterCurr = kptsCurr.at(kptMatches[i].trainIdx);
        const cv::KeyPoint &kpOuterPrev = kptsPrev.at(kptMatches[i].queryIdx);

        for (size_t j = i + 1; j < kptMatches.size(); ++j)
        {
            const cv::KeyPoint &kpInnerCurr = kptsCurr.at(kptMatches[j].trainIdx);
            const cv::KeyPoint &kpInnerPrev = kptsPrev.at(kptMatches[j].queryIdx);

            const double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            const double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            {
                distRatios.push_back(distCurr / distPrev);
            }
        }
    }

    if (distRatios.empty())
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    const size_t medianIdx = distRatios.size() / 2;
    const double medDistRatio = distRatios.size() % 2 == 0
        ? (distRatios[medianIdx - 1] + distRatios[medianIdx]) / 2.0
        : distRatios[medianIdx];

    const double dt = 1.0 / frameRate;
    TTC = -dt / (1.0 - medDistRatio);
}



void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    vector<double> prevX;
    vector<double> currX;

    for (const auto &pt : lidarPointsPrev)
    {
        prevX.push_back(pt.x);
    }
    for (const auto &pt : lidarPointsCurr)
    {
        currX.push_back(pt.x);
    }

    if (prevX.empty() || currX.empty())
    {
        TTC = NAN;
        return;
    }

    sort(prevX.begin(), prevX.end());
    sort(currX.begin(), currX.end());

    const double d0 = prevX[prevX.size() / 2];
    const double d1 = currX[currX.size() / 2];
    if (d0 <= d1)
    {
        TTC = NAN;
        return;
    }

    const double dt = 1.0 / frameRate;
    TTC = d1 * dt / (d0 - d1);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    map<pair<int, int>, int> matchCounts;

    for (const auto &match : matches)
    {
        const cv::KeyPoint &prevKpt = prevFrame.keypoints.at(match.queryIdx);
        const cv::KeyPoint &currKpt = currFrame.keypoints.at(match.trainIdx);

        vector<int> prevBoxIds;
        vector<int> currBoxIds;

        for (const auto &prevBox : prevFrame.boundingBoxes)
        {
            if (prevBox.roi.contains(prevKpt.pt))
            {
                prevBoxIds.push_back(prevBox.boxID);
            }
        }

        for (const auto &currBox : currFrame.boundingBoxes)
        {
            if (currBox.roi.contains(currKpt.pt))
            {
                currBoxIds.push_back(currBox.boxID);
            }
        }

        for (const int prevBoxId : prevBoxIds)
        {
            for (const int currBoxId : currBoxIds)
            {
                matchCounts[{prevBoxId, currBoxId}]++;
            }
        }
    }

    for (const auto &prevBox : prevFrame.boundingBoxes)
    {
        int bestCurrBoxId = -1;
        int bestCount = 0;

        for (const auto &currBox : currFrame.boundingBoxes)
        {
            const int count = matchCounts[{prevBox.boxID, currBox.boxID}];
            if (count > bestCount)
            {
                bestCount = count;
                bestCurrBoxId = currBox.boxID;
            }
        }

        if (bestCurrBoxId >= 0)
        {
            bbBestMatches[prevBox.boxID] = bestCurrBoxId;
        }
    }
}
