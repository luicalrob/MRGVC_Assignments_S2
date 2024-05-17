/*
 * SquareL.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

/**
 * \file   SquareL.cpp
 * \brief  Simulation with 30 agents navigating a square and an 'L' shape environment.
 */
#define _USE_MATH_DEFINES
#include <plotter.hpp>
#include <cmath>
#include <HRVO.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
using namespace hrvo;

std::vector <cv::Scalar> generateRandomColors(size_t numBots);
std::vector <Vector2> generateSquarePositions(size_t numBots, double sideLength);
std::vector <Vector2> generateLShapePositions(size_t numBots, double sideLength);

int main(int argc, char *argv[])
{
	// Initialization and Parameters
	srand(time(NULL));
	size_t numBots = 30;
	double stepSize = 1 / static_cast<double>(numBots);
	size_t windowSize = 1000;
	double scaleFactor = 8;
	size_t botSize = 5;
	double sideLength = 60.0f;
	std::vector <cv::Scalar> botColors = generateRandomColors(numBots);
	plotter display(windowSize, windowSize, scaleFactor, botSize);

	std::vector <Vector2> squarePositions = generateSquarePositions(numBots, sideLength);
	std::vector <Vector2> lShapePositions = generateLShapePositions(numBots, sideLength);
	// Environment Definition

	Simulator simulator;
	simulator.setTimeStep(0.25f);
	simulator.setAgentDefaults(15.0f, 10, 1.f, 1.5f, 1.0f, 2.0f);

	for (std::size_t i = 0; i < numBots; ++i) {
		simulator.addAgent(squarePositions[i], simulator.addGoal(lShapePositions[i]));
	}

	double posX, posY;
	do {
		for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
			posX = simulator.getAgentPosition(i).getX();
			posY = simulator.getAgentPosition(i).getY();
			display.drawRobot(posX, posY, botColors[i]);
		}
		display.showAndClearImage();
		simulator.doStep();
	} while (!simulator.haveReachedGoals());

	bool isVisible = true;
	while (isVisible)
	{
		cv::waitKey(0);
		isVisible = cv::getWindowProperty(display.get_window_name(), cv::WND_PROP_VISIBLE) != 0;
	}
	return 0;
}

///////////////
// Functions //
///////////////

// Generates random colors for the robots
std::vector <cv::Scalar> generateRandomColors(size_t numBots){
	std::vector <cv::Scalar> colors;
	double R, G, B;
	for (size_t i = 0; i<numBots; i ++)
	{
		R = rand()%225 + 30;
		G = rand()%225 + 30;
		B = rand()%225 + 30;
		colors.push_back(cv::Scalar(R,G,B));
	}
	return colors;
}

// Generates initial positions along a square centered at 0,0
std::vector <Vector2> generateSquarePositions(size_t numBots, double sideLength){
	std::vector <Vector2> positions;
	Vector2 currPos = Vector2(sideLength/2, sideLength/2);
	positions.push_back(currPos);
	double step = sideLength * 4 / ((float) numBots);
	
	std::vector <Vector2> directions;
	directions.push_back(Vector2( 0,-1)*step); // 1st go down
	directions.push_back(Vector2(-1, 0)*step); // 2nd go left
	directions.push_back(Vector2( 0, 1)*step); // 3rd go up
	directions.push_back(Vector2( 1, 0)*step); // 4rd go right

	std::vector <double> lengths;
	lengths.push_back(sideLength); 			
	lengths.push_back(sideLength);
	lengths.push_back(sideLength);
	lengths.push_back(sideLength);
	uint j = 0; // keeps track of the direction
	double alreadyAdvanced = 0;

	for (size_t i; i < numBots-1; i++){
		currPos += directions[j]; // move in the current direction
		alreadyAdvanced += step;
		if (alreadyAdvanced > lengths[j]){
			currPos -= directions[j]; // go back
			currPos += directions[j]/step * (step - (alreadyAdvanced-lengths[j])); // advance until the end of the side
			j += 1; // change directions
			currPos += directions[j]/step * (alreadyAdvanced-lengths[j-1]); // advance the rest of the step in the new direction
			alreadyAdvanced -= lengths[j-1]; // restart the counter of the advancement on the new side
		}
		positions.push_back(currPos);
	}
	return positions;
}


std::vector <Vector2> generateLShapePositions(size_t numBots, double sideLength){
    std::vector <Vector2> positions;

    Vector2 currPos = Vector2(-sideLength/4, sideLength/2);
    positions.push_back(currPos);

    double step = (sideLength + sideLength*3/4) / ((float) numBots);

    std::vector <Vector2> directions;
    directions.push_back(Vector2( 0,-1)*step); // 1st go down
    directions.push_back(Vector2( 1, 0)*step); // 2nd go right
    std::vector <double> lengths;
    lengths.push_back(sideLength);            // vertical is as tall as square
    lengths.push_back(sideLength*3/4);        // horizontal is 3/4 the length of the vertical
    uint j = 0; // keeps track of the direction
    double alreadyAdvanced = 0;

    for (size_t i = 0; i < numBots-1; i++) {
        currPos += directions[j]; // move in the current direction
        alreadyAdvanced += step;
        if (alreadyAdvanced > lengths[j]) {
            currPos -= directions[j]; // go back
            currPos += directions[j] / step * (step - (alreadyAdvanced - lengths[j])); // advance until the end of the side
            j += 1; // change directions
            currPos += directions[j] / step * (alreadyAdvanced - lengths[j-1]); // advance the rest of the step in the new direction
            alreadyAdvanced -= lengths[j-1]; // restart the counter of the advancement on the new side
        }
        positions.push_back(currPos);
    }
    return positions;
}
