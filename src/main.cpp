#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // instance  car
  Car car;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy,
               &car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message
    // event. The 4 signifies a websocket message The 2 signifies a
    // websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          car.x = j[1]["x"];
          car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = j[1]["yaw"];
          car.speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          // of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car.s = end_path_s;
          }

          /**
           * Planning
           * Collision-free > Obstacle-free > Legal trajectories > Viable >
           * Comfortable > Efficiency
           **/
          actions next_action = car.planning(sensor_fusion, prev_size);

          switch (next_action) {
            case A_TURN_LEFT:
              car.curr_lane -= 1;
              break;

            case A_TURN_RIGHT:
              car.curr_lane += 1;
              break;

            case A_ACCELERATE:
              car.ref_vel += .224;  //  5 m/s^2
              break;

            case A_BRAKE:
              car.ref_vel -= .224;  //  5 m/s^2
              break;

            default:
              break;
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car.x;
          double ref_y = car.y;
          double ref_yaw = deg2rad(car.yaw);

          // if previous size is almost empty, use the car as starting
          // reference
          if (prev_size < 2) {
            // generate two points that make the path tangent to the car
            double prev_car_x = car.x - cos(car.yaw);
            double prev_car_y = car.y - sin(car.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car.y);

          }
          // use the previous path's end point as starting reference
          else {
            // redefine reference state as previous path and point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting
          // reference
          vector<double> next_wp0 =
              getXY(car.s + 30, (2 + 4 * car.curr_lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car.s + 60, (2 + 4 * car.curr_lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car.s + 90, (2 + 4 * car.curr_lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // do a transformation to this local car's coordinates, we shift
          // it so we make sure that car that last point of the previous
          // path is at (0, 0), the origin and its angles at 0 degrees.
          // Makes the math easier.

          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set(x, y) points to the spline
          s.set_points(ptsx, ptsy);

          // define the acutal (x, y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at
          // our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist =
              sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with
          // previous points, here we will always output 50 points
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            // dividing it by 2.24 because this was in miles per hour, it
            // needs to be in meters per second.
            double N = (target_dist / (.02 * car.ref_vel / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it eariler local  to
            // global coordinates
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

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