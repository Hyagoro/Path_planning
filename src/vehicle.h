//
// Created by steve on 14/04/18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

    double ref_velocity;

    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_speed;
    double car_yaw;

    int lane;

    float target_speed;
    int lanes_available = 3;

    string state;

    /**
    * Constructor
    */
    Vehicle();
    Vehicle(int lane, string state, double ref_velocity);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void choose_next_state(vector<vector<double>> sensor_fusion, int prev_size);

    vector<string> successor_states();

    void generate_trajectory(const string &state, vector<vector<double>> sensor_fusion, int prev_size);

    void keep_lane_trajectory(string state, vector<vector<double>> sensor_fusion, int prev_size);

    void lane_change_trajectory(string state);

    void prep_lane_change_trajectory(string state);

    double calculate_cost(string state, vector<vector<double>> sensor_fusion, int prev_size);

};

#endif //PATH_PLANNING_VEHICLE_H
