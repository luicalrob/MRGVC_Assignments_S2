/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Mapping/LocalMapping.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"
#include <iterator>

using namespace std;

LocalMapping::LocalMapping() {

}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame, int &nMPs) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    //Remove redundant MapPoints
    mapPointCulling();

    //Triangulate new MapPoints
    triangulateNewMapPoints();

    checkDuplicatedMapPoints();

    //Run a local Bundle Adjustment
    localBundleAdjustment(pMap_.get(),currKeyFrame_->getId());

    nMPs = pMap_->getMapPoints().size();
}

void LocalMapping::mapPointCulling() {
    /*
     * Your code for Lab 4 - Task 4 here!
     */

    std::list<std::shared_ptr<MapPoint>>::iterator lit = mlpRecentAddedMapPoints.begin();

    int currentKF_id = currKeyFrame_->getId();
    int accepted_0 = 0;
    int rejected_ratio = 0;
    int rejected_2 = 0;
    int accepted = 0;

    //int borrar = mlpRecentAddedMapPoints.size();

    while(lit != mlpRecentAddedMapPoints.end()) {
        
        std::shared_ptr<MapPoint> pMP = *lit;

        int mpID = (int)pMP->getId();

        size_t mpPosition = std::distance(mlpRecentAddedMapPoints.begin(), lit);

        int firstKeyframeID = pMap_->firstKeyFrameOfMapPoint(mpID);
        int nObservations = pMap_->getNumberOfObservations(mpID);
        
        if(firstKeyframeID == -1) {  // pointmap deleted
            auto nextLit = mlpRecentAddedMapPoints.erase(lit);
            lit = nextLit; 
            accepted_0++;
        } else {

            shared_ptr<KeyFrame> KFToCheck = pMap_->getKeyFrame(firstKeyframeID);
            int nKFMapPoints = KFToCheck->getNummberOfMapPoints();
            int KFID = KFToCheck->getId();

            vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(KFID);
            int nvisible = 0;
            int nsameview = 0;

            float percentageOfSimilar = 0.0;

            for(int i = 0; i < vKFcovisible.size(); i++){
                float percentageOfSimilar = static_cast<float>(pMap_->numberOfCommonObservationsBetweenKeyFrames(KFID, vKFcovisible[i].first)) / nKFMapPoints;
                if(pMap_->isMapPointInKeyFrame(mpID, vKFcovisible[i].first)!=-1) {
                    nsameview++;
                    nvisible++;
                }else if(percentageOfSimilar >= 0.7) {
                    nsameview++;
                }
            }
            float foundRatio = static_cast<float>(nvisible)/nsameview;

            if(foundRatio < 0.25){
                pMap_->removeMapPoint(mpID);
                auto nextLit = mlpRecentAddedMapPoints.erase(lit);
                lit = nextLit; 
                rejected_ratio++;
            }else if((currentKF_id-firstKeyframeID)>=2 && nObservations<=2) {
                pMap_->removeMapPoint(mpID);
                auto nextLit = mlpRecentAddedMapPoints.erase(lit);
                lit = nextLit; 
                rejected_2++;
            }else if((currentKF_id-firstKeyframeID)>=3) {
                auto nextLit = mlpRecentAddedMapPoints.erase(lit);
                lit = nextLit; 
                accepted++;
            }
        }
        lit++;

    }
}

void LocalMapping::triangulateNewMapPoints() {
    //Get a list of the best covisible KeyFrames with the current one
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());

    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    //Get data from the current KeyFrame
    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(pKF->getId() == currKeyFrame_->getId())
            continue;

        //Check that baseline between KeyFrames is not too short
        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01){
            continue;
        }

        Sophus::SE3f T2w = pKF->getPose();

        Sophus::SE3f T21 = T2w*T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        //Match features between the current and the covisible KeyFrame
        //TODO: this can be further improved using the orb vocabulary
        int nMatches = searchForTriangulation(currKeyFrame_.get(),pKF.get(),settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(),E,vMatches);

        vector<cv::KeyPoint> vTriangulated1, vTriangulated2;
        vector<int> vMatches_;
        //Try to triangulate a new MapPoint with each match
        for(size_t i = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                /*
                 * Your code for Lab 4 - Task 2 here!
                 * Note that the last KeyFrame inserted is stored at this->currKeyFrame_
                 */
                shared_ptr<CameraModel> calibration2 = pKF->getCalibration();
                auto x1 = currKeyFrame_->getKeyPoint(i).pt;
                auto x2 = pKF->getKeyPoint(vMatches[i]).pt;

                Eigen::Vector3f xn1 = calibration1->unproject(x1).normalized();
                Eigen::Vector3f xn2 = calibration2->unproject(x2).normalized();
                Eigen::Vector3f x3D;

                triangulate(xn1, xn2, T1w, T2w, x3D);

                //Check positive depth
                auto x_1 = T1w * x3D;
                auto x_2 = T2w * x3D;
                if(x_1[2] < 0.0 || x_2[2] < 0.0) continue;

                //Check parallax
                auto ray1 = (T1w.inverse().rotationMatrix() * xn1).normalized();
                auto ray2 = (T2w.inverse().rotationMatrix() * xn2).normalized();
                auto parallax = cosRayParallax(ray1, ray2);

                if(parallax > settings_.getMinCos()) continue;

                //Check reprojection error

                Eigen::Vector2f p_p1;
                Eigen::Vector2f p_p2;
                calibration1->project(T1w*x3D, p_p1);
                calibration2->project(T2w*x3D, p_p2);

                cv::Point2f cv_p1(p_p1[0], p_p1[1]);
                cv::Point2f cv_p2(p_p2[0], p_p2[1]);

                auto e1 = squaredReprojectionError(x1, cv_p1);
                auto e2 = squaredReprojectionError(x2, cv_p2);

                if(e1 > 5.991 || e2 > 5.991) continue;

                std::shared_ptr<MapPoint> map_point(new MapPoint(x3D));

                pMap_->insertMapPoint(map_point);
                mlpRecentAddedMapPoints.push_back(map_point);

                pMap_->addObservation(currKeyFrame_->getId(), map_point->getId(), i);
                pMap_->addObservation(pKF->getId(), map_point->getId(), vMatches[i]);

                currKeyFrame_->setMapPoint(i, map_point);
                pKF->setMapPoint(vMatches[i], map_point);

                nTriangulated++;
            }
        }
    }
}

void LocalMapping::checkDuplicatedMapPoints() {
    vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<shared_ptr<MapPoint>> vCurrMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vKFcovisible.size(); i++){
        if(vKFcovisible[i].first == currKeyFrame_->getId())
            continue;
        int nFused = fuse(pMap_->getKeyFrame(vKFcovisible[i].first),settings_.getMatchingFuseTh(),vCurrMapPoints,pMap_.get());
        pMap_->checkKeyFrame(vKFcovisible[i].first);
        pMap_->checkKeyFrame(currKeyFrame_->getId());
    }
}
