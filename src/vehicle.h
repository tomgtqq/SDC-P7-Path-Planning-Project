#ifndef VEHICLE_H
#define VEHICLE_H

#include <cmath>
#include <string>
#include <vector>
using std::string;
using std::vector;

/**  Finite State Machines (FSM)
 *  READY startup
 *  LANE_KEEP
 *  LANE_CHANGE_LEFT
 *  LANE_CHANGE_RIGHT
 **/
enum states {
  S_LANE_KEEP,
  S_PRE_LANE_CHANGE_LEFT,
  S_PRE_LANE_CHANGE_RIGHT,
  S_LANE_CHANGE_LEFT,
  S_LANE_CHANGE_RIGHT
};

const vector<string> state = {"S_LANE_KEEP", "S_PRE_LANE_CHANGE_LEFT",
                              "S_PRE_LANE_CHANGE_RIGHT", "S_LANE_CHANGE_LEFT",
                              "S_LANE_CHANGE_RIGHT"};

/**  Actions
 *  A_KEEP        Keep current status
 *  A_ACCELERATE
 *  A_BRAKE
 *  A_TURN_LEFT
 *  A_TURN_RIGHT
 **/
enum actions { A_KEEP, A_ACCELERATE, A_BRAKE, A_TURN_LEFT, A_TURN_RIGHT };

class Car {
 public:
  // Constructors
  Car(){};

  // sensor fusion data
  vector<vector<double>> sensor_data;

  // Car states
  double x, y, s, d, yaw, speed;

  // Car in the lane
  int curr_lane = 1;

  // startup
  double ref_vel = 0.0;

  // safe distance
  double safe_distance = 20.0;

  // speed limit
  double speed_limit = 49.0;

  actions planning(const vector<vector<double>> &sensor_data, int prev_size);

 private:
  // FSM
  vector<states> fsm = {S_LANE_KEEP};
  actions next_action = A_KEEP;
};

#endif  // VEHICLE_H