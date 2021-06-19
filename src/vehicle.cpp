#include "vehicle.h"

#include <iostream>
#include <vector>
using std::vector;

actions Car::planning(const vector<vector<double>> &sensor_data,
                      int prev_size) {
  // if under speed limit , accelerate in highway
  next_action = ref_vel < speed_limit ? A_ACCELERATE : A_KEEP;

  bool left_safe = true;
  bool right_safe = true;

  for (int i = 0; i < sensor_data.size(); i++) {
    float d = sensor_data[i][6];

    double vx = sensor_data[i][3];
    double vy = sensor_data[i][4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_data[i][5];
    // check car feature location
    check_car_s += ((double)prev_size * .02 * check_speed);

    if (d < (2 + 4 * curr_lane + 2) && d > (2 + 4 * curr_lane - 2)) {
      if ((check_car_s > s) && (check_car_s - s < safe_distance)) {
        std::cout << state.at(fsm.back()) << std::endl;
        switch (fsm.back()) {
          case S_LANE_KEEP:
            // overtake from left lane low cost
            if (curr_lane > 0) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_LEFT);
              break;
            }
            // overtake from right lane high cost
            if (curr_lane < 2) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_LEFT);
              break;
            }
            // for other statues
            next_action = A_BRAKE;
            break;

          case S_PRE_LANE_CHANGE_LEFT:
            next_action = A_BRAKE;
            if (curr_lane == 0) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_RIGHT);
            }
            break;

          case S_PRE_LANE_CHANGE_RIGHT:
            next_action = A_BRAKE;
            if (curr_lane == 2) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_LEFT);
            }
            break;

          case S_LANE_CHANGE_LEFT:
            next_action = A_BRAKE;
            fsm.pop_back();
            fsm.push_back(S_LANE_KEEP);
            break;

          case S_LANE_CHANGE_RIGHT:
            next_action = A_BRAKE;
            fsm.pop_back();
            fsm.push_back(S_LANE_KEEP);
            break;

          default:
            next_action = A_BRAKE;
            fsm.pop_back();
            fsm.push_back(S_LANE_KEEP);
            break;
        }
      }
    }

    // left lane safe
    if ((d < (2 + 4 * (curr_lane - 1) + 2) &&
         d > (2 + 4 * (curr_lane - 1) - 2))) {
      if (curr_lane > 0) {
        if (fabs(check_car_s - s) > 3.0 * safe_distance &&
            fsm.back() == S_PRE_LANE_CHANGE_LEFT) {
          next_action = A_TURN_LEFT;
          if (left_safe) {
            next_action = A_TURN_LEFT;
          }
          fsm.pop_back();
          fsm.push_back(S_LANE_CHANGE_LEFT);
        } else if (fabs(check_car_s - s) < 3.0 * safe_distance) {
          left_safe &= false;
        }
      }
    }

    // right lane safe
    if (d < (2 + 4 * (curr_lane + 1) + 2) &&
        d > (2 + 4 * (curr_lane + 1) - 2)) {
      if (curr_lane < 2) {
        if (fabs(check_car_s - s) > 3.0 * safe_distance &&
            fsm.back() == S_PRE_LANE_CHANGE_RIGHT) {
          if (right_safe) {
            next_action = A_TURN_RIGHT;
          }
          fsm.pop_back();
          fsm.push_back(S_LANE_CHANGE_RIGHT);
        } else if (fabs(check_car_s - s) < 3.0 * safe_distance) {
          right_safe &= false;
        }
      }
    }
  }
  return next_action;
}
