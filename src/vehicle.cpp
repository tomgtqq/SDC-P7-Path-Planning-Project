#include "vehicle.h"

#include <iostream>
#include <vector>

using std::vector;

actions Car::planning(const vector<vector<double>> &sensor_data,
                      int prev_size) {
  // accelerate in highway if under speed limit
  next_action = ref_vel < speed_limit ? A_ACCELERATE : A_KEEP;

  for (int i = 0; i < sensor_data.size(); i++) {
    float d = sensor_data[i][6];
    double vx = sensor_data[i][3];
    double vy = sensor_data[i][4];
    double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = sensor_data[i][5];
    // check car feature location
    check_car_s += ((double)prev_size * .02 * check_speed);

    // check same lane
    if (d < (2 + 4 * curr_lane + 2) && d > (2 + 4 * curr_lane - 2)) {
      if ((check_car_s > s) && (check_car_s - s < safe_distance)) {
        std::cout << state.at(fsm.back()) << std::endl;
        switch (fsm.back()) {
          case S_LANE_KEEP:
            // initial safe
            left_safe = true;
            right_safe = true;

            // scan left and right lane if safe
            if (curr_lane > 0 && !select) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_LEFT);
              select = curr_lane == 2 ? false : !select;
            } else if (curr_lane < 2 && select) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_RIGHT);
              select = curr_lane == 0 ? true : !select;
            }
            next_action = A_BRAKE;
            break;

          case S_PRE_LANE_CHANGE_LEFT:
            if (left_safe == false) {
              fsm.pop_back();
              fsm.push_back(S_LANE_KEEP);
              next_action = A_BRAKE;
            } else {
              fsm.pop_back();
              fsm.push_back(S_LANE_CHANGE_LEFT);
            }
            break;

          case S_PRE_LANE_CHANGE_RIGHT:
            if (right_safe == false) {
              fsm.pop_back();
              fsm.push_back(S_LANE_KEEP);
              next_action = A_BRAKE;
            } else {
              fsm.pop_back();
              fsm.push_back(S_LANE_CHANGE_RIGHT);
            }
            break;

          case S_LANE_CHANGE_LEFT:
            break;

          case S_LANE_CHANGE_RIGHT:
            break;

          default:
            break;
        }
      }
    }
    // left lane safe
    else if (d < (2 + 4 * (curr_lane - 1) + 2) &&
             d > (2 + 4 * (curr_lane - 1) - 2) && curr_lane > 0) {
      if (curr_lane > 0 && fsm.back() == S_PRE_LANE_CHANGE_LEFT) {
        if (fabs(check_car_s - s) < 1.5 * safe_distance) {
          left_safe &= false;
        } else if (fabs(check_car_s - s) > 3.0 * safe_distance) {
          left_safe &= true;
        }
      }
    }
    // right lane safe
    else if (d < (2 + 4 * (curr_lane + 1) + 2) &&
             d > (2 + 4 * (curr_lane + 1) - 2) && curr_lane < 2) {
      if (curr_lane < 2 && fsm.back() == S_PRE_LANE_CHANGE_RIGHT) {
        if (fabs(check_car_s - s) < 1.5 * safe_distance) {
          right_safe &= false;
        } else if (fabs(check_car_s - s) > 3.0 * safe_distance) {
          right_safe &= true;
        }
      }
    }
  }

  // change lane
  if (fsm.back() == S_LANE_CHANGE_LEFT && left_safe == true) {
    next_action = A_TURN_LEFT;
    fsm.pop_back();
    fsm.push_back(S_LANE_KEEP);
  } else if (fsm.back() == S_LANE_CHANGE_RIGHT && right_safe == true) {
    next_action = A_TURN_RIGHT;
    fsm.pop_back();
    fsm.push_back(S_LANE_KEEP);
  }

  std::cout << n_act.at(next_action) << std::endl;
  return next_action;
}
