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
    void UpdateSensorFusion(vector<vector<double>> & sensorFusion, double car_s,double car_d,uint prevPlanLeftover);
    double GetTargetVelocity(void) const;
    uint   GetTargetLane(void) const;

private:

    bool LaneIsClear(vector<vector<double>> & sensorFusion, double car_s, uint prevPlanLeftove, uint lane, double safeDistance ) const;
    bool LaneSwitchIsSafe(vector<vector<double>> & sensorFusion, double car_s, uint prevPlanLeftove, uint lane, double safeDistance ) const;

    void HandleObstcaleInSwitching(vector<vector<double>> & sensorFusion, double car_s,double car_d,uint prevPlanLeftover);
    void HandleObstcaleInKeepLane(vector<vector<double>> & sensorFusion, double car_s,double car_d,uint prevPlanLeftover);


    const double  POINTS_FREQ = 0.02;
    const double  LANE_WIDTH = 4.0; //in meters
    const double  MAX_LEGAL_VELOCITY = 49.9;
    const double  SPEED_INCREMENT = 1.5;
    const double  INITIAL_SPPED = 0.0;
    const double  SAFE_DISTANCE = 30; //in meters
    const double  REAR_SAFTY = 10; //in meters

    enum DrivingStates{
        KEEP_LANE,
        SWITCH_LANE_RIGHT,
        SWITCH_LANE_LEFT
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
    state_ = KEEP_LANE;
};
void DrivingStateMachine::UpdateSensorFusion(vector<vector<double>> & sensorFusion, double car_s,double car_d,uint prevPlanLeftover)
{
    bool obstacleDetected = false;

    // Check if we finished switching lane
    if(state_ == DrivingStates::SWITCH_LANE_LEFT || state_ == DrivingStates::SWITCH_LANE_RIGHT )
    {
         if((car_d<(LANE_WIDTH/2+LANE_WIDTH*targetLane_+LANE_WIDTH/3))&&(car_d>(LANE_WIDTH/2+LANE_WIDTH*targetLane_-LANE_WIDTH/3)))
        {
            state_=KEEP_LANE;
            currentLane_ = targetLane_;
        }  
     }

 	if (!LaneIsClear(sensorFusion,car_s,prevPlanLeftover,targetLane_,SAFE_DISTANCE))
    {
        
        switch(state_){
            case DrivingStates::SWITCH_LANE_LEFT:
            case DrivingStates::SWITCH_LANE_RIGHT:
            {
                // Swirl back to original lane (current lane)
                // switching back to KEEP LANE state
                HandleObstcaleInSwitching(sensorFusion, car_s, car_d, prevPlanLeftover);
                break;
            }
            case DrivingStates::KEEP_LANE:
            {
                // Reduce speed - move to PREPARE_SWITCH_LANE_
                // switching back to KEEP LANE state
                HandleObstcaleInKeepLane(sensorFusion, car_s, car_d, prevPlanLeftover);
                break;
            }
            
          }
            
    }
    else if(referenceVelocity_ < MAX_LEGAL_VELOCITY )
    {
       referenceVelocity_ += min(SPEED_INCREMENT,MAX_LEGAL_VELOCITY-referenceVelocity_);
    }
   
    
}

bool DrivingStateMachine::LaneIsClear(vector<vector<double>> & sensorFusion, double car_s, uint prevPlanLeftover, uint lane,double safeDistance ) const
{
    //scan all cars in sensor fusion
	for( auto sensced_car : sensorFusion)
	{
		double d = sensced_car[6];
		//if car is in our target lane - we will check if it is too close
		if((d<(LANE_WIDTH/2+LANE_WIDTH*lane+LANE_WIDTH/2))&&(d>(LANE_WIDTH/2+LANE_WIDTH*lane-LANE_WIDTH/2)))
		{
			double vx = sensced_car[3];
			double vy = sensced_car[4];

			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensced_car[5];

			//predictin where sensced car will be when the previous plan end
			check_car_s +=((double)prevPlanLeftover*POINTS_FREQ*check_speed);

			if((check_car_s> car_s)&& ((check_car_s - car_s) < safeDistance))
			{
				return  false;
			}
		}
	}
    return true;
}

bool DrivingStateMachine::LaneSwitchIsSafe(vector<vector<double>> & sensorFusion, double car_s, uint prevPlanLeftover, uint lane,double safeDistance ) const
{
    //scan all cars in sensor fusion
	for( auto sensced_car : sensorFusion)
	{
		double d = sensced_car[6];
		//if car is in our target lane - we will check if it is too close
		if((d<(LANE_WIDTH/2+LANE_WIDTH*lane+LANE_WIDTH/2))&&(d>(LANE_WIDTH/2+LANE_WIDTH*lane-LANE_WIDTH/2)))
		{
			double vx = sensced_car[3];
			double vy = sensced_car[4];

			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensced_car[5];

			//predictin where sensced car will be when the previous plan end
			check_car_s +=((double)prevPlanLeftover*POINTS_FREQ*check_speed);

			if(((check_car_s> car_s)&& ((check_car_s - car_s) < safeDistance))||
                    ((check_car_s<= car_s)&& (( car_s- check_car_s) < REAR_SAFTY)))
			{
				return  false;
			}
		}
	}
    return true;
}
void DrivingStateMachine::HandleObstcaleInKeepLane(vector<vector<double>> & sensorFusion, double car_s,double car_d,uint prevPlanLeftover)
{
    // In case of an obstacle in current  lane in keep lane we will try to make the care pass from the left
    // if path not clear, we will prepare to pass by slowing down and moving state to prepare for lane switch
    // Passing on the right will be done only when we need to pass from the left most lane
    if(currentLane_ > 0)
    {
        if(LaneSwitchIsSafe(sensorFusion, car_s, prevPlanLeftover, currentLane_ -1,SAFE_DISTANCE))
        {
           state_ = SWITCH_LANE_LEFT;
           targetLane_ = currentLane_-1;
        } 
    }
    if(state_ == KEEP_LANE && currentLane_ +1 < numberOfLanes_)
    {
        if(LaneSwitchIsSafe(sensorFusion, car_s, prevPlanLeftover, currentLane_ +1,SAFE_DISTANCE))
        {
            state_ = SWITCH_LANE_RIGHT;
            targetLane_ = currentLane_+1;
        }

    }
    if(state_ == KEEP_LANE )
    {
        referenceVelocity_ -= SPEED_INCREMENT;  
    }   
}

void DrivingStateMachine::HandleObstcaleInSwitching(vector<vector<double>> & sensorFusion, double car_s,double car_d,uint prevPlanLeftover)
{
    referenceVelocity_ -= SPEED_INCREMENT;
}

double DrivingStateMachine::GetTargetVelocity(void) const
{
    return referenceVelocity_;
}

uint   DrivingStateMachine::GetTargetLane(void) const
{
    return targetLane_;
}
    



