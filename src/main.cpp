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
#include "hybrid_breadth_first.h"
#include "spline.h"
#include "tools.h"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

bool isToClose(vector<vector<double>> sensor_fusion, int my_lane, int prev_size, double car_s) {
    bool too_close = false;
    // find ref_v to use
    for (auto &car_details : sensor_fusion) {
        // car is in ly lane
        double d = car_details[6];
        int his_lane = ((int) d / 4);
//        std::cout << "d : " << d << " his_lane : " << his_lane << std::endl;
        if(his_lane == my_lane) { //ex lane 1 : 8 > d > 4
            double vx = car_details[3];
            double vy = car_details[4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = car_details[5];

            check_car_s += ((double)prev_size) * 0.02 * check_speed;

            if((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
//                                ref_vel = 29.5;
                too_close = true;
            }
        }
    }
    return too_close;
}

void computeNextPoints(Vehicle vehicle,
                       int prev_size, vector<double> previous_path_x, vector<double> previous_path_y,
                       const vector<double> &map_waypoints_x,
                       const vector<double> &map_waypoints_y,
                       const vector<double> &map_waypoints_s,
                       vector<double> &next_x_vals, vector<double> &next_y_vals) {
    vector<double> x_points;
    vector<double> y_points;


    double ref_x = vehicle.car_x;
    double ref_y = vehicle.car_y;
    double ref_yaw = deg2rad(vehicle.car_yaw);



    if(prev_size < 2) {
        double prev_car_x = vehicle.car_x - cos(vehicle.car_yaw);
        double prev_car_y = vehicle.car_y - sin(vehicle.car_yaw);

        x_points.push_back(prev_car_x);
        x_points.push_back(vehicle.car_x);

        y_points.push_back(prev_car_y);
        y_points.push_back(vehicle.car_y);
    }
    else {

        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        x_points.push_back(ref_x_prev);
        x_points.push_back(ref_x);

        y_points.push_back(ref_y_prev);
        y_points.push_back(ref_y);
    }

    vector<double> next_wp0 = getXY(vehicle.car_s + 30, (2 + 4 * vehicle.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(vehicle.car_s + 60, (2 + 4 * vehicle.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(vehicle.car_s + 90, (2 + 4 * vehicle.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    x_points.push_back(next_wp0[0]);
    x_points.push_back(next_wp1[0]);
    x_points.push_back(next_wp2[0]);

    y_points.push_back(next_wp0[1]);
    y_points.push_back(next_wp1[1]);
    y_points.push_back(next_wp2[1]);

    assert(x_points.size() == y_points.size());
    for(int i = 0; i < x_points.size(); i++) {
        //shift car angle to 0 degrees
        double shift_x = x_points[i] - ref_x;
        double shift_y = y_points[i] - ref_y;

        x_points[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        y_points[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create spline
    tk::spline s;

    // set(x,y) to the spline
    s.set_points(x_points, y_points);



    // Start with all of the previous path points form the last time
    for(int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points,
    // here we will always output

    for(int i = 1; i <= 50 - previous_path_x.size(); i++) {
        double N = (target_dist / (0.02 * vehicle.ref_velocity / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;


        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
//        std::cout << "x_point = " << x_point << std::endl;
//        std::cout << "y_point = " << y_point << std::endl;
    }
}

int main() {
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

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
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

//    double ref_vel = 0; //mph
//    int lane = 1;

    Vehicle vehicle(1, "KL", 0);

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &vehicle](
            uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
            uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (!s.empty()) {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    vehicle.car_x = j[1]["x"];
                    vehicle.car_y = j[1]["y"];
                    vehicle.car_s = j[1]["s"];
                    vehicle.car_d = j[1]["d"];
                    vehicle.car_yaw = j[1]["yaw"];
                    vehicle.car_speed = j[1]["speed"];

//                    std::cout << "car_x = " << vehicle.car_x << std::endl;
//                    std::cout << "car_y = " << vehicle.car_y << std::endl;

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];


                    int prev_size = previous_path_x.size();


                    if(prev_size > 0) {
                        vehicle.car_s = end_path_s;
                    }

//                    bool too_close = isToClose(sensor_fusion, vehicle.lane, prev_size, vehicle.car_s);
                    // to close check


                    vehicle.choose_next_state(sensor_fusion, prev_size);

                    // Define the actual (x,y)
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // compute next points for build trajectory
                    computeNextPoints(vehicle,
                            prev_size, previous_path_x, previous_path_y,
                            map_waypoints_x, map_waypoints_y, map_waypoints_s,
                            next_x_vals, next_y_vals);

                    json msgJson;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

//                    std::cout << "msgJson = " << msgJson << std::endl;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

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

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}