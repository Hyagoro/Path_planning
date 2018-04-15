//
// Created by steve on 14/04/18.
//

#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>

//#include "tools.h"
#include "vehicle.h"

/**
 * Initializes Vehicle
 */
const double ACCELERATION = 0.3;
const double SPEED = 49.5;

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, string state, double ref_velocity) {

    this->lane = lane;
    this->state = state;
    this->ref_velocity = ref_velocity;
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}


void Vehicle::choose_next_state(vector<vector<double>> sensor_fusion, int prev_size) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();

    string best_state;
    double prev_cost = 1000000.0;
    for (auto &state : states) {
//        vector<Vehicle> trajectory = generate_trajectory(state, predictions);
//        if (!trajectory.empty()) {
            double cost = calculate_cost(state, sensor_fusion, prev_size);
//            costs.push_back(cost);
//            final_trajectories.push_back(trajectory);
        if(cost < prev_cost) {
            best_state = state;
            prev_cost = cost;
        }

    }
    std::cout << std::endl;

//    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
//    int best_idx = distance(begin(costs), best_cost);
//    return final_trajectories[best_idx];
    generate_trajectory(best_state, sensor_fusion, prev_size);
//    return ;

}


double Vehicle::calculate_cost(string state, vector<vector<double>> sensor_fusion, int prev_size) {
    double cost = 0.0;

    bool car_ahead = false;
    bool car_at_left = false;
    bool car_at_right = false;

    for (auto &car_details : sensor_fusion) {
        // car is in ly lane
        double d = car_details[6];
        int his_lane = ((int) d / 4);
        int front_distance = 20;
        int back_distance = 10;

        double vx = car_details[3];
        double vy = car_details[4];
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = car_details[5];
        check_car_s += ((double) prev_size) * 0.02 * check_speed;

        if (his_lane == this->lane) {
            if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                car_ahead = true;
            }
        }

        else if(his_lane == (this->lane - 1)) {
            if ((this->car_s - back_distance < check_car_s) && (this->car_s + front_distance > check_car_s)) {
                car_at_left = true;
            }
        }

        else if(his_lane == (this->lane + 1)) {
            if ((this->car_s - back_distance < check_car_s) && (this->car_s + front_distance > check_car_s)) {
                car_at_right = true;
            }
        }
    }

    /*
         *  "KL" - Keep Lane
            "LCL" / "LCR"- Lane Change Left / Lane Change Right
            "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right
         */

    if(!car_ahead) {
        if(state == "KL"){
            cost = 0;
        }
        if(state == "PLCL"){
            cost = 10;
        }
        if(state == "PLCR"){
            cost = 10;
        }
    }
    // if car ahead
    else {
        if(car_at_left) {
            if(state == "LCL" || state == "PLCL") {
                cost += 100;
            }
        }
        else{
            if(state == "LCL") {
                cost += -20;
            }
            else {
                cost += -10;
            }
        }
        if(car_at_right) {
            if(state == "LCR" || state == "PLCR")
            cost += 100;
        }
        else {
            if(state == "LCR") {
                cost += -20;
            }
            else {
                cost += -10;
            }
        }
        if(state == "KL"){
            cost += 10;
        }
    }

    std::cout << "car_ahead = " << car_ahead << " car_at_left = " << car_at_left << " car_at_right : "
    << car_at_right;
    std::cout << " actual state = " << this->state << " req state = " << state << " cost : "
              << cost << " lane : " << this->lane <<  std::endl;

    return cost;
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.

    "KL" - Keep Lane
    "LCL" / "LCR"- Lane Change Left / Lane Change Right
    "PLCL" / "PLCR" - Prepare Lane Change Left / Prepare Lane Change Right
    */
    vector<string> states;
    states.emplace_back("KL");
    string state = this->state;
    if(state == "KL") {
        if (lane != 0) {
            states.emplace_back("PLCL");
        }
        if (lane != lanes_available - 1) {
            states.emplace_back("PLCR");
        }
    } else if (state == "PLCL") {
        if (lane != 0) {
            states.emplace_back("PLCL");
            states.emplace_back("LCL");
        }
    } else if (state == "PLCR") {
        if (lane != lanes_available - 1) {
            states.emplace_back("PLCR");
            states.emplace_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

void Vehicle::generate_trajectory(const string &state, vector<vector<double>> sensor_fusion, int prev_size) {
    /*
     TODO Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
//    vector<Vehicle> trajectory;
//    if (state.compare("CS") == 0) {
//        trajectory = constant_speed_trajectory();
//    } else
    if (state == "KL") {
        keep_lane_trajectory(state, sensor_fusion, prev_size);
    } else if (state == "LCL" || state == "LCR") {
        lane_change_trajectory(state, sensor_fusion);
    } else if (state == "PLCL" || state == "PLCR") {
        prep_lane_change_trajectory(state, sensor_fusion);
    }
}

//
//vector<Vehicle> Vehicle::constant_speed_trajectory() {
//    /*
//    Generate a constant speed trajectory.
//    */
//    float next_pos = position_at(1);
//    vector<Vehicle> trajectory = {Vehicle(this->lane, this->state, 0),
//                                  Vehicle(this->lane, this->state, 0)};
//    return trajectory;
//}

void Vehicle::keep_lane_trajectory(string state, vector<vector<double>> sensor_fusion, int prev_size) {
    /*
    Generate a keep lane trajectory.
    */
    bool too_close = false;
    for (auto &car_details : sensor_fusion) {
        // car is in ly lane
        double d = car_details[6];
        int his_lane = ((int) d / 4);
        if (his_lane == this->lane) {
            double vx = car_details[3];
            double vy = car_details[4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = car_details[5];

            check_car_s += ((double) prev_size) * 0.02 * check_speed;

            if ((check_car_s > car_s) && ((check_car_s - car_s) < 20)) {
                too_close = true;
            }
        }
    }
    if(too_close) {
        this->ref_velocity -= ACCELERATION * 0.70;
    }
    else if(this->ref_velocity < SPEED) {
        this->ref_velocity += ACCELERATION;
    }
    this->state = state;
}

void Vehicle::prep_lane_change_trajectory(string state, vector<vector<double>> sensor_fusion) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    this->state = state;
}

void Vehicle::lane_change_trajectory(string state, vector<vector<double>> sensor_fusion) {
    /*
    Generate a lane change trajectory.
    */
    if(state == "LCL") {
        this->lane -= 1;
        this->state = state;
    }
    else if(state == "LCR") {
        this->lane += 1;
        this->state = state;
    }
}
