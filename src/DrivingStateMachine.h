/*
 * DrivingStateMachine.h
 *
 * simple driving state machine implimentation 
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2018 Yitshak Yarom (yitshak.yaorm at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 */

#pragma once

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

class DrivingStateMachine{
    
public:

    DrivingStateMachine(uint initialLane = 1, uint numberOfLanes = 3);
    void UpdateSensorFusion(vector<vector<double>> sensorFusion, double car_s,double car_d,uint prevPlanLeftover);
    double GetTargetVelocity(void) const;
    uint   GetTargetLane(void) const;

private:

    const double  POINTS_FREQ = 0.02;
    const double  LANE_WIDTH = 4.0; //in meters
    const double  MAX_LEGAL_VELOCITY = 49.5;
    const double  SPEED_INCREMENT = 0.224;
    const double  INITIAL_SPPED = 5.0;
    const double  SAFE_DISTANCE = 30;

    enum DrivingStates{
        KEEP_LANE,
        SWITCH_LANE_RIGHT,
        SWITCH_LANE_LEFT,
        PREPARE_SWITCH_LANE_RIGHT,
        PREPARE_SWITCH_LANE_LEFT,
    };

    DrivingStates state_;
    uint currentLane_;
    uint targetLane_;
    uint numberOfLanes_;

    double referenceVelocity_;
};


DrivingStateMachine::DrivingStateMachine(uint initialLane, uint numberOfLanes)
{
    currentLane_ = targetLane_ = initialLane;
    numberOfLanes_ = numberOfLanes;
    referenceVelocity_ = DrivingStateMachine::INITIAL_SPPED;
};
    
void DrivingStateMachine::UpdateSensorFusion(vector<vector<double>> sensorFusion, double car_s,double car_d,uint prevPlanLeftover)
{
    bool collision = false;
    //scan all cars in sensor fusion
	for( auto sensced_car : sensorFusion)
	{
		double d = sensced_car[6];
		//if car is in our lane - we will check if it is too close
		if((d<(LANE_WIDTH/2+LANE_WIDTH*currentLane_+LANE_WIDTH/2))&&(d>(LANE_WIDTH/2+LANE_WIDTH*currentLane_-LANE_WIDTH/2)))
		{
			double vx = sensced_car[3];
			double vy = sensced_car[4];

			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensced_car[5];

			//predictin where sensced car will be when the previous plan end
			check_car_s +=((double)prevPlanLeftover*POINTS_FREQ*check_speed);

			if((check_car_s> car_s)&& ((check_car_s - car_s) < DrivingStateMachine::SAFE_DISTANCE))
			{
				collision =  true;
			}
		}
	}
	if (collision)
    {
        referenceVelocity_ -= DrivingStateMachine::SPEED_INCREMENT;
    }
    else if(referenceVelocity_ < DrivingStateMachine::MAX_LEGAL_VELOCITY)
    {
        referenceVelocity_ += DrivingStateMachine::SPEED_INCREMENT;
    }
    
}

double DrivingStateMachine::GetTargetVelocity(void) const
{
    return referenceVelocity_;
}

uint   DrivingStateMachine::GetTargetLane(void) const
{
    return targetLane_;
}
    



