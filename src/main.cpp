#include "carState.h"
#include "map.h"
#include "stateMachine.h"
#include "trajectoryPlanner.h"
#include "utilities.h"
#include "worldModel.h"

#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "json.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
inline string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");

	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}

	return "";
}


int main(int argc_, char **argv_)
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	Map map;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

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
		map.waypoints_x.push_back(x);
		map.waypoints_y.push_back(y);
		map.waypoints_s.push_back(s);
		map.waypoints_dx.push_back(d_x);
		map.waypoints_dy.push_back(d_y);
	}

	WorldModel worldModel (map);
	StateMachine stateMachine (worldModel);
	TrajectoryPlanner trajectoryPlanner (worldModel);

	if (argc_ > 1)
	{
		switch (argv_[1][0])
		{
		case 't':
			break;
		}
	}

	h.onMessage([&map, &worldModel, &stateMachine, &trajectoryPlanner]
	            (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
	{
		auto const time = std::chrono::steady_clock::now();
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string const event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					CarState car;

					car.x = j[1]["x"];
					car.y = j[1]["y"];
					car.s = j[1]["s"];
					car.d = j[1]["d"];
					car.yaw = deg2rad(j[1]["yaw"]);
					car.speed = mph2m_s(j[1]["speed"]);

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					for (auto const &t : sensor_fusion)
					{
						WorldModel::Target nt (t[0], t[1], t[2], t[3], t[4], t[5], t[6], time);

						worldModel.update (nt);
					}

					Maneuver const desiredManeuver = stateMachine.update (car);

					json msgJson;

					Trajectory const trajectory =
					trajectoryPlanner.update (previous_path_x, previous_path_y,
					end_path_s, end_path_d, car, desiredManeuver);

					msgJson["next_x"] = trajectory.x;
					msgJson["next_y"] = trajectory.y;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
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
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
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

	int const port = 4567;

	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
