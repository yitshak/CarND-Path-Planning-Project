# Model documentation

As describe in the course, there are two main components for path planning:
- Behavior selection
- Trajectory generation

In general, it is recommended to separate  behavior selection from trajectory generation, both in source and in context (behavior selection is invoked less often than trajectory creation). However, for simplicity in this project, I have only separated the modules but their invocation is done from the same context (message sent from simulator).

Once a message is  received from simulator, DrivingStateMachine is updated with sensor fusion data (and other data needed) and makes a decision about whether it is needed to change  lane and accordingly sets :state, target velocity and target lane. 

After this update, our main message handler runs the sequence to generate smooth trajectory which is sent back to the simulator.

This sequence is reflected in main.cpp:

```c++
stateMachine.UpdateSensorFusion(sensor_fusion,car_s,car_d,previous_size);
ref_velocity = stateMachine.GetTargetVelocity();
lane = stateMachine.GetTargetLane();

// List of way points for path

vector<double> anchor_points_x;
vector<double> anchor_points_y;
                   
double ref_x = current_x;
double ref_y = current_y;
double ref_yaw = deg2rad(current_yaw);

// set the first two points 
setInitialPoints(previous_path_x, previous_path_y, anchor_points_x,anchor_points_y,ref_x,ref_y,ref_yaw);
                    
// add 3 points evenly spaced 30 m 
addSpacedPoints(map_waypoints_x, map_waypoints_y, map_waypoints_s, 3, 30.0,lane,car_s,anchor_points_x,anchor_points_y);
                
     
// transform the points according to refference point to simplify math of calculating spline
alignPointsWithRef( ref_x, ref_y, ref_yaw, anchor_points_x, anchor_points_y);

vector<vector<double>> next_values = 
                        calculateNextValues(previous_path_x,previous_path_y,anchor_points_x,anchor_points_y,ref_x,ref_y,ref_yaw,ref_velocity/2.24);
```
                


## Behavior selection

Behavior selection in this solution uses a _very_ simple state machine which has only 3 states: Keep lane(KL),Switch to right lane(SR) and Switch to left lane (SL). 

The state machine doesn’t out put it’s state bu use it to determine target velocity and target lane.

The default state is KL, and while lane is clear we stay in this state. When an obstacle is detected state machine checks if left lane is clear for passing. If it is clear, than state machine switches state to SL, if not checks right lane for SR, and if right lane is not clear as well state will remain KL but target velocity will be reduced.

If Obstacle is detected while in SR or SL state, model will only reduce speed and won’t try to change state.

When switching lane is completed (identified by state machine) state turns back to Keep lane.

## Trajectory generation

After destination lane and reference velocity is set by our state machine a sequence to generate trajectory is run. This sequence is very similar to the one shown in the Q&A session for this project. To make it easier to follow and debug, I have separated the flow to several methods.

Initially anchor points are generated (5 points), according to this points a spline is generated using spline.h as recommended. After spline is ready, we create the final path. Final path is composed of "leftover" from last path planned and samples from new spline created. Using "leftover" from last path ensures we have a more smooth path and also reduces the number of new points we need to calculate.

- *setInitialPoints* - Sets the initial anchor points according to the last two points of the previous path. If there are not enough points in previous path we generate this two points from car current position and a point "behind" the car that will make path adjacent to car.

- *addSpacedPoints* - As discussed in class we add 3 points that are 30 m aprat (s dimmension) according to our destination lane.

- *alignPointsWithRef* - This method transforms the points according car, so that all pointes are relative to car. This makes the math easier and ensures that "spline.h" gives us variable results.

- *calculateNextValues* - from the anchor points create spline. After adding the "leftover" points, sample points from spline (so that keeps velocity reference by correct spacing as explained in Q&A) and add to path.

  
## Possible Improvements
The main module that can be improved is the state machine that implements the behavior model:

1. Add more states (preparation states) - Currently if we cannot pass safely on either side we just slow down. Adding preparation stages will allow setting an optimal velocity to safly pass in the future.
2. Decide to pass before car is too close - Currently we check if car can pass only when object is very close, so we need to  either slowdown or pass. We can check for opportunity to pass before we need to slow down to avoid collision.
3. Keep to middle lane - since we are only looking on adjacent lanes for passing an obstacle, the model should strive for car to be in the middle lane, and switch to middle lane when ever possible.
4. Checking speed of other objects - currently we do not consider actual speed of other objects (Cars), if we did we could choose our next state more optimally.


