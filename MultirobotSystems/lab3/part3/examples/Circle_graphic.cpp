/*
 * CircleSimulation.cpp
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
 * \file   CircleSimulation.cpp
 * \brief  Example simulation of agents navigating through a circular environment.
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

std::vector <cv::Scalar> generateColors(size_t numBots);

int main(int argc, char *argv[])
{
	// Initialization and Parameters
	size_t numBots = 56;
    size_t windowWidth = 1000;
	double scaleFactor = 8;
	size_t botSize = 5;
	double circleRadius = 40.0f;
	double stepSize = 1 / static_cast<double>(numBots);


	std::vector <cv::Scalar> botColors = generateColors(numBots);
	plotter display(windowWidth, windowWidth, scaleFactor, botSize);

	// Define Environment
	Simulator sim;
	sim.setTimeStep(0.25f);
	sim.setAgentDefaults(15.0f, 10, 1.f, 1.5f, 1.0f, 2.0f);

	for (std::size_t i = 0; i < numBots; ++i) {
		const Vector2 pos = circleRadius * Vector2(std::cos(stepSize * i * HRVO_TWO_PI), std::sin(stepSize * i * HRVO_TWO_PI));
		sim.addAgent(pos, sim.addGoal(-pos));
	}

	double posX, posY;
	do {
		for (std::size_t i = 0; i < sim.getNumAgents(); ++i) {
			posX = sim.getAgentPosition(i).getX();
			posY = sim.getAgentPosition(i).getY();
			display.drawRobot(posX, posY, botColors[i]);
		}
		display.showAndClearImage();
		sim.doStep();
	} while (!sim.haveReachedGoals());
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

std::vector<cv::Scalar> generateColors(size_t numBots) {
    std::vector<cv::Scalar> colors;
    int fraction = numBots / 4;

    for (size_t i = 0; i < fraction; i++) {
        int greenComponent = 180; // Green component between 100 and 250
        colors.push_back(cv::Scalar(0, greenComponent, 0)); // RGB format: (B, G, R)
    }
    for (size_t i = fraction; i < fraction * 2; i++) {
        int redComponent = 180; // Red component between 100 and 250
        colors.push_back(cv::Scalar(redComponent, 0, 0)); // RGB format: (B, G, R)
    }
    for (size_t i = fraction * 2; i < fraction * 3; i++) {
        int blueComponent = 180; // Blue component between 100 and 250
        colors.push_back(cv::Scalar(0, 0, blueComponent)); // RGB format: (B, G, R)
    }
    for (size_t i = fraction * 3; i < numBots; i++) {
        int greenComponent = 180; // Green component between 100 and 250
        int blueComponent = 180; // Blue component between 100 and 250
        colors.push_back(cv::Scalar(0, greenComponent, blueComponent)); // RGB format: (B, G, R)
    }
    return colors;
}
