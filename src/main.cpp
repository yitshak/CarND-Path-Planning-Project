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

#include "DrivingStateMachine.h"

using namespace std;

#define POINTS_FREQ 0.02
#define LANE_WIDTH 4 //in meters
#define MAX_LEGAL_VELOCITY 49.5
#define SPEED_INCREMENT 0.224
#define INITIAL_SPPED 5
#define SAFE_DISTANCE 30

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = min(2 * pi() - angle, angle);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

/*
Add initial points ro path
*/
void addInitialPointsToPath(vector<double> &anchor_points_x, vector<double> &anchor_points_y, double current_x, double current_y, double current_yaw)
{
	anchor_points_x.push_back(current_x - cos(current_yaw));
	anchor_points_y.push_back(current_y - sin(current_yaw));

	anchor_points_x.push_back(current_x);
	anchor_points_y.push_back(current_y);
}



/*
This funtion set two initial points to path (anchor_points_x,anchor_points_y) according to state of previoues path
and update ref_x,ref_y, ref_yaw accordingly.

ref_x,ref_y, ref_yaw should be set with current x,y,yaw before calling this function
*/
void setInitialPoints(const vector<double> & previous_path_x, 
	const vector<double> & previous_path_y,
	vector<double> & anchor_points_x,
	vector<double> & anchor_points_y,
	double & ref_x,
	double & ref_y,
	double & ref_yaw)
{

	// TO ensure we have inly two point at the end of function
	anchor_points_x.clear();
	anchor_points_y.clear();
	
	// initialy we set first point for initial path according to current position and yaw
	// so that path is ajecent to car current position
	double ref_x_previous = ref_x - cos(ref_yaw);
	double ref_y_previous = ref_y - sin(ref_yaw);
	
	// if there are enough points left in previoues path to form direction (2), start path planing from 
	// last point of previoues path, we change ref accordingly
	if (previous_path_x.size() > 2)
	{
		//We start our planing from last point in previoues path
		//so we need to update reference
		uint last_index = previous_path_x.size() -1;
		ref_x = previous_path_x[last_index];
		ref_y = previous_path_y[last_index];

		ref_x_previous = previous_path_x[last_index-1];
		ref_y_previous = previous_path_y[last_index-1];

		//using two last points we calculate the ref yaw
		ref_yaw = atan2(ref_y-ref_y_previous,ref_x-ref_x_previous);
	}

	// we push the two seelected points as first points in our new path
	anchor_points_x.push_back(ref_x_previous);
	anchor_points_y.push_back(ref_y_previous);

	anchor_points_x.push_back(ref_x);
	anchor_points_y.push_back(ref_y);

	
}

/*
Add to point_x and anchor_points_y evenly spaced points according to parameters
*/
void addSpacedPoints(const vector<double> & map_waypoints_x, 
	const vector<double> & map_waypoints_y, 
	const vector<double> & map_waypoints_s,
	uint num_of_points, 
	double distance,
	uint lane,
	double ref_s,
	vector<double> & anchor_points_x,
	vector<double> & anchor_points_y)
{
	for(uint i =1; i<= num_of_points;i++)
	{
		vector<double> next_way_point = getXY(ref_s+i*distance,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		anchor_points_x.push_back(next_way_point[0]);
		anchor_points_y.push_back(next_way_point[1]);
	}
	for(uint i =0; i< anchor_points_x.size();i++)
	{
		//cout << __FUNCTION__<< ":" << anchor_points_x[i] << "::" << anchor_points_y[i] << std::endl;
	
	}
	
}

/*
Transform all points according to reference x,y and yaw
*/
void alignPointsWithRef( double ref_x, 
	double ref_y,
	double ref_yaw,
	vector<double> & anchor_points_x,
	vector<double> & anchor_points_y)
{

	for(uint i = 0;i<anchor_points_x.size();i++)
	{
		//calculate shift
		double x_shift = anchor_points_x[i]-ref_x;
		double y_shift = anchor_points_y[i]-ref_y;

		anchor_points_x[i] = (x_shift*cos(0-ref_yaw)) - (y_shift*sin(0-ref_yaw));
		anchor_points_y[i] = (x_shift*sin(0-ref_yaw)) + (y_shift*cos(0-ref_yaw));

		//cout << __FUNCTION__<< ":" << anchor_points_x[i] << "::" << anchor_points_y[i] << "  ref  :"<< ref_x << ',' << ref_y << ',' << ref_yaw << std::endl;
	}
}

vector<vector<double>> calculateNextValues(const vector<double> & previous_path_x,
	const vector<double> & previous_path_y,
	const vector<double> & anchor_points_x,
	const vector<double> & anchor_points_y,
	double ref_x,
	double ref_y,
	double ref_yaw,
	double target_velocity) //in m/s
{
	vector<double> next_x_values;
	vector<double> next_y_values;

	//fWe will add new points to points left from last planing
	for(uint i=0;i<previous_path_x.size();i++)
	{
		next_x_values.push_back(previous_path_x[i]);
		next_y_values.push_back(previous_path_y[i]);
	} 


	//Calculating new points using spline
	tk::spline path_spline;
	path_spline.set_points(anchor_points_x,anchor_points_y);
	
	//set Horizon fo planning
	double horizon_x = 30.0;
	double horizon_y = path_spline(horizon_x);
	double horizon_distance = sqrt((horizon_x*horizon_x+horizon_y*horizon_y));
	// Set number of points according to speed
	double N = (horizon_distance/(POINTS_FREQ*(target_velocity)));
	double x_increment = horizon_x/N;

	double x_add_on = 0.0;
	for(uint i=0;i<50-previous_path_x.size();i++)
	{
		double x = x_add_on +x_increment;
		double y = path_spline(x);

		x_add_on = x;

		// Need to transform x,y to global reference

		double x_temp = x;
		double y_temp = y;

		x = (x_temp*cos(ref_yaw)-y_temp*sin(ref_yaw));
		y = (x_temp*sin(ref_yaw)+y_temp*cos(ref_yaw));

		x+=ref_x;
		y+=ref_y;

		next_x_values.push_back(x);
		next_y_values.push_back(y);



	}
	
	return {next_x_values,next_y_values};
} 


int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	double ref_velocity = 0; //mph
	uint 	lane = 1;		//Start lane
	DrivingStateMachine stateMachine;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	
	

	h.onMessage([&stateMachine, &ref_velocity,&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																																																					 uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;

		
		
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					double current_x = j[1]["x"];
					double current_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double current_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					vector<double> previous_path_x = j[1]["previous_path_x"];
					vector<double> previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

					uint previous_size = previous_path_x.size();

					if(previous_size >0 )
					{
						car_s = end_path_s;
						car_d = end_path_d;
					}
					
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

					json msgJson;
					msgJson["next_x"] = next_values[0];
					msgJson["next_y"] = next_values[1];

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
										 size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
												 char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
