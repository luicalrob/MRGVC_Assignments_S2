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

/*
 * Author: Juan J. Gómez Rodríguez (jjgomez@unizar.es)
 *
 * A demo showing the Mini-SLAM library processing a sequence of the TUM dataset
 */

#include "DatasetLoader/TUMRGBDLoader.h"
#include "System/MiniSLAM.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv){
    //Check program parameters are good
    if(argc != 3){
        cerr << "[Error]: you need to invoke the program with 2 parameters: " << argc << endl;
        cerr << "\t./mono_tumrgbd <dataset_path>" << endl;
        cerr << "\ttrajectory file name" << endl;
        cerr << "Finishing execution..." << endl;
        return -1;
    }

    //Load dataset sequence
    string datasetPath = argv[1];
    TUMRGBDLoader sequence(datasetPath, datasetPath + "/rgb.txt");

    //Create SLAM system
    MiniSLAM SLAM("Data/TUMRGBD.yaml");

    //File to store the trajectory
    string trajectoryFileName = argv[2];
    ofstream trajectoryFile (trajectoryFileName + ".txt");
    if (!trajectoryFile.is_open()){
        cerr << "[Error]: could not open the trajectory file, aborting..." << endl;
        return -2;
    }

    clock_t timer;
	timer = clock();
    int nKF = 0;

    //Process the sequence
    cv::Mat currIm;
    double currTs;
    for(int i = 0; i < sequence.getLenght(); i++){
        sequence.getRGBImage(i,currIm);
        sequence.getTimeStamp(i,currTs);

        Sophus::SE3f Tcw;
        if(SLAM.processImage(currIm, Tcw, nKF, timer)){
            Sophus::SE3f Twc = Tcw.inverse();
            //Save predicted pose to the file
            trajectoryFile << setprecision(17) << currTs << "," << setprecision(7) << Twc.translation()(0) << ",";
            trajectoryFile << Twc.translation()(1) << "," << Twc.translation()(2) << ",";
            trajectoryFile << Twc.unit_quaternion().x() << "," << Twc.unit_quaternion().y() << ",";
            trajectoryFile << Twc.unit_quaternion().z() << "," << Twc.unit_quaternion().w() << endl;
        }
    }

    trajectoryFile.close();

    timer = clock() - timer;
	cout << "[END] Seconds: " << fixed << setprecision(4) << ((float)timer)/CLOCKS_PER_SEC << endl;
    cout << "[END] Number of Keyframes: " << nKF << endl;

    return 0;
}

