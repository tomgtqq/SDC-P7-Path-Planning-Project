#include "vehicle.h"

#include <iostream>
#include <vector>

using std::vector;

actions Car::planning(const vector<vector<double>> &sensor_data,
                      int prev_size) {
  std::cout << "-------------------------------------------------" << std::endl;
  // if under speed limit , accelerate in highway
  next_action = ref_vel < speed_limit ? A_ACCELERATE : A_KEEP;

  bool left_safe = true;
  bool right_safe = true;

  bool checked_left = false;
  bool checked_right = false;

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

            if (curr_lane > 0 && !checked_left) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_LEFT);
              std::cout << "55555555555555555" << std::endl;
            } else if (curr_lane < 2 && !checked_right) {
              fsm.pop_back();
              fsm.push_back(S_PRE_LANE_CHANGE_RIGHT);
              std::cout << "66666666666666666" << std::endl;
            }

            next_action = A_BRAKE;
            std::cout << "77777777777777777" << std::endl;

            break;

          case S_PRE_LANE_CHANGE_LEFT:
            std::cout << "8888888888888888888" << std::endl;
            if (left_safe == false) {
              fsm.pop_back();
              fsm.push_back(S_LANE_KEEP);
              checked_left = true;
              checked_right = false;
              next_action = A_BRAKE;
              std::cout << "ccccccccccccccccc" << std::endl;
            } else {
              fsm.pop_back();
              fsm.push_back(S_LANE_CHANGE_LEFT);
              std::cout << "eeeeeeeeeeeeeeeeeee" << std::endl;
            }
            break;

          case S_PRE_LANE_CHANGE_RIGHT:
            std::cout << "9999999999999999999" << std::endl;
            if (right_safe == false) {
              fsm.pop_back();
              fsm.push_back(S_LANE_KEEP);
              checked_right = true;
              checked_left = false;
              next_action = A_BRAKE;
              std::cout << "dddddddddddddddddd" << std::endl;
            } else {
              fsm.pop_back();
              fsm.push_back(S_LANE_CHANGE_RIGHT);
              std::cout << "ffffffffffffffffff" << std::endl;
            }
            break;

          case S_LANE_CHANGE_LEFT:
            // if (left_safe == false) {
            //   next_action = A_BRAKE;
            //   return next_action;
            // }
            std::cout << "AAAAAAAAAAAAAAAAA" << std::endl;
            break;

          case S_LANE_CHANGE_RIGHT:
            // if (right_safe == false) {
            //   next_action = A_BRAKE;
            //   return next_action;
            // }
            std::cout << "BBBBBBBBBBBBBBBBBB" << std::endl;
            break;

          default:
            // next_action = A_BRAKE;
            // fsm.pop_back();
            // fsm.push_back(S_LANE_KEEP);
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
          std::cout << "111111111111111111 left_safe -->" << left_safe
                    << std::endl;
        } else if (fabs(check_car_s - s) > 3.0 * safe_distance) {
          left_safe &= true;
          std::cout << "222222222222222222 left_safe -->" << left_safe
                    << std::endl;
        }
      }
    }

    // right lane safe
    else if (d < (2 + 4 * (curr_lane + 1) + 2) &&
             d > (2 + 4 * (curr_lane + 1) - 2) && curr_lane < 2) {
      if (curr_lane < 2 && fsm.back() == S_PRE_LANE_CHANGE_RIGHT) {
        if (fabs(check_car_s - s) < 1.5 * safe_distance) {
          right_safe &= false;
          std::cout << "333333333333333333 right_safe -->" << right_safe
                    << std::endl;
        } else if (fabs(check_car_s - s) > 3.0 * safe_distance) {
          right_safe &= true;
          std::cout << "444444444444444444 right_safe -->" << right_safe
                    << std::endl;
        }
      }
    }
  }

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
  std::cout << "*************************************************" << std::endl;
  return next_action;
}
