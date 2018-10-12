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

    DrivingStateMachine(uint initialLane, uint numberOfLanes);
    void UpdateSensorFusion(vector<vector<double>> sensorFusion, double car_s,double car_d);
    double GetTargetVelocity(void) const;
    uint   GetTargetLane(void) const;

private:
    enum DrivingStates{
        KEEP_LANE,
        SWITCH_LANE_RIGHT,
        SWITCH_LANE_LEFT,
        PREPARE_SWITCH_LANE_RIGHT,
        PREPARE_SWITCH_LANE_LEFT,
    };
};


DrivingStateMachine::DrivingStateMachine(uint initialLane, uint numberOfLanes)
{}
    
void DrivingStateMachine::UpdateSensorFusion(vector<vector<double>> sensorFusion, double car_s,double car_d)
{}

double DrivingStateMachine::GetTargetVelocity(void) const
{}

uint   DrivingStateMachine::GetTargetLane(void) const
{}
    



